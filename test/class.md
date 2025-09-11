## クラスとは
変数や関数をまとめて管理するもの
以下は例(あくまで例)
~~~
class Servo //クラスの宣言 class class名
{  
    void set_PIN(int a){
        pin = a;
    }
    //ここからは適当な関数や変数
    bool move(double x){
        //ここでx度動かすプログラム
    }
}
int main(){//多分ここはvoid loop()
    Servo servoL,servoR;    //Servoをインスタンス化(クラスから作ったものを実体にする)

    const int PIN_L = 9;    //PIN番号を宣言
    const int PIN_R = 10;

    servoL.set_PIN(PIN_L);  //PIN番号を上のset_PIN関数に入れる
    servoL.set_PIN(PIN_R);

    servoL.move(100);       //leftを100度動かす
    return 0;
}
~~~