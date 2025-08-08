import time
from jaka_control.ag95_extension import ExtendedAG95


if __name__ == '__main__':
    """
    AG95は、設定把持力を小さくすると、開閉速度も小さくなる。
    コマンドを送信して最初はグリッパの開閉速度は小さいが徐々に大きくなり最後に小さくなる。
    
    コマンドは同期ではなく非同期で常に0.08秒程度だと考えられる。

    VRで連続で動かすのではなくとぎれとぎれで送ると遅延がないという事実から何が言えるか

    コマンドを送信して最初はグリッパの開閉速度は小さいが徐々に大きくなり最後に小さくなるので、
    ちいさなへんかのコマンドを逐一送ると速度はずっと小さいままになる。
    それは以下で計測した速度よりもさらに小さくなる。

    これはいわばグリッパーはサーボモード・スレーブモードのように送信値をそのまま
    制御に送るわけではないことに由来すると思われる。

    適度な間隔で送るしかない。
    
    把持力: 20% (45 N)の場合

    gripper.read_state()=1
    gripper.set_force(20)=None
    gripper.read_force()=20
    gripper.read_pos()=1000
    gripper.set_pos(0)=None
    t_elapsed_set_pos=0.08035564608871937
    read_pos=999
    t_elapsed_read_pos=0.08037133002653718
    read_pos=997
    t_elapsed_read_pos=0.08033226407133043
    read_pos=992
    t_elapsed_read_pos=0.08029824192635715
    read_pos=984
    t_elapsed_read_pos=0.08042561612091959
    read_pos=973
    t_elapsed_read_pos=0.08039482496678829
    read_pos=960
    t_elapsed_read_pos=0.08033615490421653
    read_pos=944
    t_elapsed_read_pos=0.08028326602652669
    read_pos=925
    t_elapsed_read_pos=0.08030541916377842
    read_pos=903
    t_elapsed_read_pos=0.0803563620429486
    read_pos=878
    t_elapsed_read_pos=0.08032417297363281
    read_pos=852
    t_elapsed_read_pos=0.08043926605023444
    read_pos=821
    t_elapsed_read_pos=0.0803400450386107
    read_pos=789
    t_elapsed_read_pos=0.0803803640883416
    read_pos=754
    t_elapsed_read_pos=0.08037595497444272
    read_pos=716
    t_elapsed_read_pos=0.08029494900256395
    read_pos=675
    t_elapsed_read_pos=0.080439348006621
    read_pos=631
    t_elapsed_read_pos=0.080298637971282
    read_pos=585
    t_elapsed_read_pos=0.08031812496483326
    read_pos=536
    t_elapsed_read_pos=0.08041819999925792
    read_pos=484
    t_elapsed_read_pos=0.08051134087145329
    read_pos=429
    t_elapsed_read_pos=0.08030322287231684
    read_pos=372
    t_elapsed_read_pos=0.08042671391740441
    read_pos=312
    t_elapsed_read_pos=0.08031198801472783
    read_pos=249
    t_elapsed_read_pos=0.0803318559192121
    read_pos=184
    t_elapsed_read_pos=0.08029934391379356
    read_pos=119
    t_elapsed_read_pos=0.08038360509090126
    read_pos=54
    t_elapsed_read_pos=0.08024637098424137
    read_pos=7
    t_elapsed_read_pos=0.08036925597116351
    read_pos=0
    t_elapsed_read_pos=0.0803221189416945
    t_elapsed_completed=2.410882650176063

    把持力: 100% (160 N)の場合

    gripper.read_state()=1
    gripper.set_force(100)=None
    gripper.read_force()=100
    gripper.read_pos()=1000
    gripper.set_pos(0)=None
    t_elapsed_set_pos=0.08036367897875607
    read_pos=993
    t_elapsed_read_pos=0.08031967608258128
    read_pos=961
    t_elapsed_read_pos=0.08036587410606444
    read_pos=900
    t_elapsed_read_pos=0.08037307392805815
    read_pos=809
    t_elapsed_read_pos=0.0804269309155643
    read_pos=687
    t_elapsed_read_pos=0.0803051779512316
    read_pos=537
    t_elapsed_read_pos=0.08026389591395855
    read_pos=355
    t_elapsed_read_pos=0.08033074182458222
    read_pos=149
    t_elapsed_read_pos=0.08050112007185817
    read_pos=17
    t_elapsed_read_pos=0.08043879992328584
    read_pos=0
    t_elapsed_read_pos=0.08036359096877277
    t_elapsed_completed=0.8841747350525111
    """
    gripper = ExtendedAG95(port='/dev/ttyUSB0')
    print(f"{gripper.read_state()=}")
    print(f"{gripper.set_force(100)=}")
    print(f"{gripper.read_force()=}")
    print(f"{gripper.read_pos()=}")

    t_start_set_pos = time.perf_counter()
    print(f"{gripper.set_pos(0)=}")
    t_elapsed_set_pos = time.perf_counter() - t_start_set_pos
    print(f"{t_elapsed_set_pos=}")
    while True:
        t_start_read_pos = time.perf_counter()
        read_pos = gripper.read_pos()
        print(f"{read_pos=}")
        t_elapsed_read_pos = time.perf_counter() - t_start_read_pos
        print(f"{t_elapsed_read_pos=}")
        if read_pos == 0:
            break
    t_elapsed_completed = time.perf_counter() - t_start_set_pos
    print(f"{t_elapsed_completed=}")
