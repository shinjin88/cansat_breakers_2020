import argparse

parser = argparse.ArgumentParser(description='ランバック制御用プログラム　2020-9')
parser.add_argument('--latitude',type=float,help='緯度の指定、なければデフォルトの値を利用する')
parser.add_argument('--longitude',type=float,help='経度の指定、なければデフォルトの値を利用する')
parser.add_argument('-m','--distance-mode',type=int,help='誘導方式が変わる距離の指定、なければデフォルトの値を利用する')
parser.add_argument('-g','--distance-goal',type=int,help='ゴール判定の距離の指定、なければデフォルトの値を利用する')
parser.add_argument('-c','--console',action='store_true',help='通信の出力をターミナルへ流す')
parser.add_argument('-a','--manual',action='store_true',help='手動での制御に切り替える')
parser.add_argument('-M','--calibration-magnet',action='store_true',help='磁気センサのキャリブレーションを行う')
parser.add_argument('-d','--disable-drill',action='store_true',help='ドリルの工程をスキップする')
parser.add_argument('-p','--disable-purge',action='store_true',help='パラシュートのパージ工程をスキップする')
parser.add_argument('-C','--disable-camera',action='store_true',help='カメラによる誘導を無効にする')
args = parser.parse_args()
print(str(args))