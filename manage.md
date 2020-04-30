# 分支管理方案

* 韩煦源的 repo 为主仓库, 大家从主仓库中 fork 到自己的账号下
* 自己的远程仓库只有 master 分支, 本地仓库有 master dev 分支, 开发在 dev 上完成, 测试完成后 merge 到本地 master 分支, push 到远程仓库
* pull request 到韩煦源 repo, 由韩煦源与邓中柱负责管理 merge
  
## 具体操作方法

* 在网页版 fork 到自己账号下
* 复制到本地 `git clone 自己账号下仓库`
* 创建 dev 分支 `git branch dev`
* 切换到 dev 分支 `git switch dev`
* 编写, 修改代码
* 切换到 master 分支 `git checkout master`
* 将 dev 分支的工作合并到 master 分支上 `git merge dev`
* 删除 dev 分支 `git branch -d dev`
* 将修改推送到远程分支上 `git push origin master`
* 网页版请求主仓库合并
