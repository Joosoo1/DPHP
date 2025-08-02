from argparse import ArgumentParser
import os
import yaml
import torch
import numpy as np
from random import sample
from copy import deepcopy
from torch import autograd
from torch.utils.data import DataLoader, random_split, Subset, Dataset
import pytorch_lightning as pl
from pytorch_lightning.callbacks import ModelCheckpoint
from datatools.trajpred_dataset import TrajpredDataset
from utils.metrics import ade, fde
from utils.tools import split_task
from models import EthPredictor


class EwcPredictor(pl.LightningModule):
    def __init__(
            self,
            lr: float = 2e-3,
            l2: float = 5e-4,
            ewc_weight: float = 1e6,
            coreset_length: int = 100,
            coreset_update_length: int = 20,
            target_mode: str = 'velocity',
            pred_horizon: int = 15,
            **kwargs
    ):
        super().__init__()
        self.predictor = EthPredictor(
            pred_horizon=pred_horizon,
            target_mode=target_mode,
        )

        # train hyperparameters
        self.lr = lr
        self.l2 = l2
        self.save_hyperparameters()

        # ewc specific
        self.ewc_weight = ewc_weight
        self.params = {n: p for n, p in self.predictor.named_parameters() if p.requires_grad}
        self.prev_tasks = []

        # coreset specific
        self.coreset_length = coreset_length
        self.coreset_update_length = coreset_update_length
        self.coreset = None

    def forward(self, x):
        return self.predictor(x)

    def training_step(self, batch, batch_idx):
        inputs, targets = batch
        preds = self.predictor(inputs)

        batch_ade = ade(preds, targets)
        batch_fde = fde(preds, targets)
        loss = batch_ade + self._compute_consolidation_loss()
        self.log('train_fde', batch_fde)
        self.log('train_ade', batch_ade)
        return loss

    def validation_step(self, batch, batch_idx):
        inputs, targets = batch
        preds = self.predictor(inputs)

        targets = targets.float()
        batch_ade = ade(preds, targets)
        batch_fde = fde(preds, targets)
        batch_cv_ade = torch.mean(inputs['cv_ade'])
        batch_cv_fde = torch.mean(inputs['cv_fde'])
        self.log('val_ade', batch_ade)
        self.log('val_fde', batch_fde)
        self.log('cv_ade', batch_cv_ade)
        self.log('cv_fde', batch_cv_fde)
        return {'val_ade': batch_ade, 'val_fde': batch_fde, 'cv_ade': batch_cv_ade, 'cv_fde': batch_cv_fde}

    def validation_epoch_end(self, outputs):
        val_ade_mean = sum([o['val_ade'] for o in outputs]) / len(outputs)
        val_fde_mean = sum([o['val_fde'] for o in outputs]) / len(outputs)
        cv_ade_mean = sum([o['cv_ade'] for o in outputs]) / len(outputs)
        cv_fde_mean = sum([o['cv_fde'] for o in outputs]) / len(outputs)
        self.log('val_ade', val_ade_mean, prog_bar=True)
        self.log('val_fde', val_fde_mean, prog_bar=True)
        self.log('cv_ade', cv_ade_mean, prog_bar=True)
        self.log('cv_fde', cv_fde_mean, prog_bar=True)

    def test_step(self, batch, batch_idx):
        inputs, targets = batch
        preds = self.predictor(inputs)

        targets = targets.float()
        batch_ade = ade(preds, targets)
        batch_fde = fde(preds, targets)

        self.log('test_ade', batch_ade)
        self.log('test_fde', batch_fde)

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.predictor.parameters(), lr=self.lr, weight_decay=self.l2)
        return optimizer

    def on_save_checkpoint(self, checkpoint) -> None:
        checkpoint['prev_tasks'] = self.prev_tasks
        checkpoint['coreset'] = self.coreset

    def on_load_checkpoint(self, checkpoint) -> None:
        self.prev_tasks = checkpoint['prev_tasks']
        if 'coreset' in checkpoint.keys():
            self.coreset = checkpoint['coreset']

    def _compute_consolidation_loss(self):
        losses = []
        if len(self.prev_tasks) == 0:
            return 0

        for t in self.prev_tasks:
            for param_name, param in self.predictor.named_parameters():
                if param_name not in t['means'].keys():
                    continue
                estimated_mean = t['means'][param_name]
                estimated_fisher = t['fishers'][param_name]
                losses.append((estimated_fisher * (param - estimated_mean) ** 2).sum())

        return (self.ewc_weight / 2) * sum(losses)

    def _update_params(self, dl, new_task):
        losses = []
        for i, batch in enumerate(dl):
            inputs, targets = batch
            output = self.predictor(inputs)
            losses.append(ade(output, targets.float()))
        losses = torch.stack(losses).mean()
        param_names, params = [], []
        for (param_name, param) in self.predictor.named_parameters():
            if param.requires_grad:
                param_names.append(param_name)
                params.append(param)
        grad_log_liklihood = autograd.grad(losses, params, allow_unused=True)
        for param_name, param, fisher in zip(param_names, params, grad_log_liklihood):
            new_task['fishers'][param_name] = fisher.data.clone() ** 2
            new_task['means'][param_name] = param.data.clone()

    def register_ewc_params(self, dataloader, task_number=None):
        new_task = {'id': task_number, 'means': {}, 'fishers': {}}
        self._update_params(dataloader, new_task)
        self.prev_tasks.append(new_task)

    def add_coreset_to_loader(self, loader):
        if not isinstance(loader, DataLoader) or not isinstance(loader.dataset, Dataset):
            return NotImplementedError

        new_loader = deepcopy(loader)

        keys = new_loader.dataset._data.keys()
        for k in keys:
            if len(new_loader.dataset._data[k].shape) > 1:
                new_loader.dataset._data[k] = np.vstack((new_loader.dataset._data[k], self.coreset[k]))
            else:
                new_loader.dataset._data[k] = np.concatenate((new_loader.dataset._data[k], self.coreset[k]))
        return new_loader

    def update_coreset(self, loader):
        if not isinstance(loader, DataLoader) or not isinstance(loader.dataset, Dataset):
            return NotImplementedError

        if self.coreset is None:
            # Fill the coreset for the first time
            self.coreset = {}
            dataset_idxs = np.random.randint(len(loader.dataset), size=self.coreset_length)
            for k, v in loader.dataset._data.items():
                self.coreset[k] = v[dataset_idxs]
        else:
            # Update only #self.coreset_update_length random examples
            dataset_idxs = np.random.randint(len(loader.dataset), size=self.coreset_update_length)
            coreset_update_idxs = np.random.randint(self.coreset_length, size=self.coreset_update_length)
            for k, v in loader.dataset._data.items():
                self.coreset[k][coreset_update_idxs] = v[dataset_idxs]

    @staticmethod
    def add_model_specific_args(parent_parser):
        parser = ArgumentParser(parents=[parent_parser], add_help=False)
        parser.add_argument("--lr", type=float, default=2e-3)
        parser.add_argument("--l2", type=float, default=5e-4)
        parser.add_argument("--ewc_weight", type=float, default=10e6)
        parser.add_argument("--target_mode", type=str, default='velocity')
        parser.add_argument("--pred_horizon", type=int, default=15)
        return parser


def cli_main():
    # ------------
    # args
    # ------------
    parser = ArgumentParser()
    parser.add_argument('--model', type=str, default='EthPredictor')
    parser.add_argument('--save_name', type=str, default='default')
    parser.add_argument('--batch_size', type=int, default=32)
    parser.add_argument('--datasets', nargs='+', default=None),
    parser.add_argument('--experiments', nargs='+', default=None),
    parser.add_argument('--tbptt', type=int, default=3),
    parser.add_argument('--frequency', type=int, default=5),
    parser.add_argument('--stride', type=int, default=20),
    parser.add_argument('--min_track_length', type=float, default=1.5),
    parser = pl.Trainer.add_argparse_args(parser)
    parser = EwcPredictor.add_model_specific_args(parser)
    args = parser.parse_args()

    save_dir = f"saves/{args.model}/{args.save_name}"
    if os.path.exists(save_dir):
        print("A trained model with the same save_name already exists!")
        return
    else:
        os.makedirs(save_dir)

    # ------------
    # Model
    # ------------
    model = EwcPredictor(**vars(args))
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(device)

    # ------------
    # data
    # ------------
    dataset = TrajpredDataset(
        model.predictor,
        **vars(args),
        train=True
    )
    dataset_train, dataset_val = split_task(dataset, val_split=0.2)
    del dataset

    dataset_test = TrajpredDataset(
        model.predictor,
        **vars(args),
        train=False
    )
    print(f'train_size = {len(dataset_train)}, val_size = {len(dataset_val)}, test_size = {len(dataset_test)}')
    train_loader = DataLoader(dataset_train, batch_size=args.batch_size, shuffle=True, num_workers=6)
    val_loader = DataLoader(dataset_val, batch_size=args.batch_size, shuffle=False, num_workers=6)
    test_loader = DataLoader(dataset_test, batch_size=args.batch_size, shuffle=False, num_workers=6)
    model.update_coreset(train_loader)

    # ------------
    # training
    # ------------
    # modelcheckpoint回调，监控“val_ade”, 保存最好的3个模型
    checkpoint_callback = ModelCheckpoint(
        monitor='val_ade',
        dirpath=save_dir,
        filename='{epoch:02d}-{step:02d}-{val_ade:.2f}',
        save_top_k=3,
        mode='min',
    )
    # 初始化pytorch lightning trainer对象
    trainer = pl.Trainer.from_argparse_args(
        args,
        default_root_dir=save_dir,
        callbacks=[checkpoint_callback],
        # 自动检测并使用可用的GPU进行训练
        gpus=torch.cuda.device_count(),
        # 使用自动学习率查找
        auto_lr_find=True,
        # 设置梯度裁剪的阈值，防止梯度爆炸
        gradient_clip_val=0.5
    )

    # 启动模型训练过程，根据给定的数据加载器来执行训练和验证循环
    trainer.fit(model, train_loader, val_loader)
    model.register_ewc_params(trainer.train_dataloader)

    # ------------
    # saving
    # ------------
    trainer.save_checkpoint(save_dir + "/final.ckpt")

    # ------------
    # testing
    # ------------
    result = trainer.test(model, test_loader)
    yaml.dump(result, open(f"{save_dir}/test_results.yaml", 'w'))


if __name__ == '__main__':
    cli_main()
