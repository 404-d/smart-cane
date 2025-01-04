# Git

1. 安装[Visual Studio Code](https://code.visualstudio.com/Download)，并进行初始化
2. 安装[Git](https://git-scm.com/downloads)
3. 注册一个[GitHub](github.com)账号，并在 GitHub 上新建一个仓库，名字随便取
4. 打开 vscode，点击左边栏的"分支图标"，点击"克隆存储库(Git Clone Repository)"，按提示登录 GitHub 账号，并选择在第二步新建的仓库，克隆位置自定义
5. 等待克隆结束，在终端(Ctrl+`)中输入以下命令：
   ```sql
   git config --global user.name "你的GitHub用户名"
   gti config --global user.email "你的GitHub邮箱"
   ```
6. 新建一个文件，并作出修改
7. 回到左边栏的"分支图标"，在"更改(Changes)"后点击"加号"即可同步全部修改；在单独的修改项后点击加号，可以只同步单一的修改，完成选择后，点击"提交(Commit)"，在弹出的文件中把想要同步的修改项前的"#"删掉并保存，点击右上角的"对勾"，等待后再点击"同步修改"
