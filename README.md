git hubをwebで開く: https://github.com/
<br> sign up で自分の名前とメールアドレスを入力 </br>

git bashをダウンロードする
ダウンロード先: https://git-scm.com/install/windows

以下、git bash上で行う

環境構築
git config --global user.name "ここにsign upで登録した名前"
git config --global user.email "ここにsign upで登録したメールアドレス"

SSHkeyの作成
ls ~/.ssh //上書を防ぐために確認
ssh-keygen -C "メールアドレス"
保存先にこだわりがなければENTERを押下
パスワードの作成　いらなければENTER
ls ~/.ssh //再び確認

git hubをwebで開き、右上にある自分のアイコンをクリックしSettingsを開く
SSH and GPG keysを押下
New SSH keyを押下し、ls ~/.sshで出たid_rsa.pubをkeyにコピーアンドペースト　titleは適当に

cansatリポジトリをダウンロード
git clone git@github.com:ThinkOK1103/cansat.git

変更の追加の仕方
git pull origin main //更新されていたら
git add . //ステージング
git commit -m "hoge" //"何かコメントをする"　　ローカルリポジトリへ
git push origin main //リモートリポジトリ
