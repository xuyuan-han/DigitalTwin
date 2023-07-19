# Branch Management Scheme

* Han Xuyuan's repo is the main repository. Everyone forks from the main repository to their own account.
* Your remote repository only has the master branch. Your local repository has master and dev branches. Development is completed on the dev branch, and after testing, it's merged into the local master branch and pushed to the remote repository.
* Pull request to Han Xuyuan's repo, which is managed and merged by Han Xuyuan and Deng Zhongzhu.

# Specific Operation Methods

* Fork to your account on the web version.
* Clone to local git clone [your account repository]
* Write and modify the code.
* Push the modifications to the remote branch git push origin master
* Request the main repository to merge on the web version.

# 分支管理方案

* 韩煦源的 repo 为主仓库, 大家从主仓库中 fork 到自己的账号下
* 自己的远程仓库只有 master 分支, 本地仓库有 master dev 分支, 开发在 dev 上完成, 测试完成后 merge 到本地 master 分支, push 到远程仓库
* pull request 到韩煦源 repo, 由韩煦源与邓中柱负责管理 merge
  
## 具体操作方法

* 在网页版 fork 到自己账号下
* 复制到本地 `git clone 自己账号下仓库`
* 编写, 修改代码
* 将修改推送到远程分支上 `git push origin master`
* 网页版请求主仓库合并
