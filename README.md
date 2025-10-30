git hubをwebで開く: https://github.com/
<br> sign up で自分の名前とメールアドレスを入力 </br>

git bashをダウンロードする
<br>ダウンロード先: https://git-scm.com/install/windows</br>

以下、git bash上で行う

環境構築
<br>git config --global user.name "ここにsign upで登録した名前"</br>
git config --global user.email "ここにsign upで登録したメールアドレス"

SSHkeyの作成
<br>ls ~/.ssh //上書を防ぐために確認</br>
ssh-keygen -C "メールアドレス"
<br>保存先にこだわりがなければENTERを押下</br>
<br>パスワードの作成　いらなければENTER</br>
<br>ls ~/.ssh //再び確認</br>

git hubをwebで開き、右上にある自分のアイコンをクリックしSettingsを開く
<br>SSH and GPG keysを押下</br>
<br>New SSH keyを押下し、ls ~/.sshで出たid_rsa.pubをkeyにコピーアンドペースト　titleは適当に</br>

cansatリポジトリをダウンロード
<br>git clone git@github.com:ThinkOK1103/cansat.git</br>

変更の追加の仕方
<br>git pull origin main //更新されていたら</br>
<br>git add . //ステージング</br>
<br>git commit -m "hoge" //"何かコメントをする"　　ローカルリポジトリへ</br>
<br>git push origin main //リモートリポジトリ</br>
