## SRRG Executor
[![pipeline status](https://gitlab.com/srrg-software/srrg2_executor/badges/master/pipeline.svg)](https://gitlab.com/srrg-software/srrg2_executor/commits/master)

Interactive shell to run and modify configurations on the fly. Viever can be attached (both locally - in shared mode - and remotely - in ros mode). Dependencies' libraries are dynamically loaded :)

###### How to run
The executor has just one executable - the `srrg2_shell`. You can either load, modify and run your configuration - using the option `-c` - or generate one on the fly and then run it. For more details run:
```sh
source <srrg2_catin_workspace_root>/devel/setup.bash
rosrun srrg2_executor srrg2_shell -h
```

Warning: to execute the shell you to have the `roscore` active in your system.

### IN DEMO VERITAS

    run sync slam

#### 2D Laser SLAM
Datasets (json)
 - [Killian Court](https://drive.google.com/open?id=1Rq_tA7ZjhBASrtHsFfHBuJmBvDs2_2xK)
 - [Intel Research Lab](https://drive.google.com/open?id=1upKPzQVnu4RxaTIYKw6YAA6WzW7GHVcG)

#### 3D Stereo SLAM
Datasets (json)
 - KITTI [00](https://drive.google.com/open?id=1u_d4qp5p7eyWQk4zZhI8x13uEB_Ut7pC),
[01](https://drive.google.com/open?id=1oPoczbCpXJyB_1lz7AsNqHu4NiWcqpkM),
[02](https://drive.google.com/open?id=1LXwdo0mGaSKBQ_Pi_1fOK-VMQx0ciAwq),
[03](https://drive.google.com/open?id=1jtZpf_u5nb_0sa3etXsLV79J5ywG14pc),
[04](https://drive.google.com/open?id=1YZFLycQp4ilSaLTrovLxSY80UK2Q2wF1),
[05](https://drive.google.com/open?id=1i5xCzLTNa0uW3RvroLQpkoLA7qPD103m),
[06](https://drive.google.com/open?id=1F6UR8ycHKGosg9YcriZvBiX2nUNvV2EL),
[07](https://drive.google.com/open?id=1gMRUpX4IOR70lB87_gzevVpq-VAsu26G),
[08](https://drive.google.com/open?id=15lg3x3KNuPYMxowE38hWBZk8ch5SdfOI),
[09](https://drive.google.com/open?id=1DtGrqxS50stlUMpeOP-gUHdWFuPp4gZg),
[10](https://drive.google.com/open?id=1-fQCLyr2IT7H-l_EXfQtJ8H9nRhr1lZD)
 - Malaga [00](https://drive.google.com/open?id=1u_d4qp5p7eyWQk4zZhI8x13uEB_Ut7pC),
[01](https://drive.google.com/open?id=14igPWcnSMduW_7X5A5KwJnO4hXwkMEwq),
[02](https://drive.google.com/open?id=1WpPS9GEeRJJ_fjjoALQdNd4gNpQEUU0a),
[03](https://drive.google.com/open?id=1sjLYtO4Bf7Brhh2_iQw3AERcLZ-y4P2Z),
[04](https://drive.google.com/open?id=1UtuUVkC3CUap8G6Lw-gca6yIebpzmqHJ),
[05](https://drive.google.com/open?id=1bTpLfH1V0X9yiZn8eG-RMtQMjLruB2EV),
[06](https://drive.google.com/open?id=1PySxzqj1e3RP4nnSIvNpCXdeqg-C6-Zp),
[07](https://drive.google.com/open?id=1_lj7x4xdoMfX6ydDJBCzPhnXbIv_URyu),
[08](https://drive.google.com/open?id=1UkLUoasfBNvTshhKMAAjJcHUNQZfaZa0),
[09](https://drive.google.com/open?id=14XC14qszpI5Fnc-hKQW8obmxLk3FooAj),
[10](https://drive.google.com/open?id=1xuHSBlVtTAgzZosaTNZlKmCN6zgYGIjB),
[11](https://drive.google.com/open?id=1qGZWlpgSlEIJcC83Br7yPvRnujD__jOq),
[12](https://drive.google.com/open?id=1Bpkw7VWRrPFdvRL_YcLpkPLDQZ29rRP8),
[13](https://drive.google.com/open?id=1BwREeo5b9_px5BXxqXSrsly_NQQz9D0g),
[14](https://drive.google.com/open?id=1LxpWeM_0kc9xxP-ovsQ_NUivnY-mMqEu),
[15](https://drive.google.com/open?id=1Lf09DEILSzUkHVcdGfelbq2cqgUs0zfJ)
- EuRoC [MH_01](https://drive.google.com/open?id=1g6_QudgZetukSTe6MwYxauaei_8n_lza),
[MH_02](https://drive.google.com/open?id=1XzSHNZtEnt67D9Twb_kOwuA-cSlgB7LE),
[MH_03](https://drive.google.com/open?id=1mG30GL0WvClVWpDxWkOtCwWvFgNcK3Bv),
[MH_04](https://drive.google.com/open?id=1ylbX7EdKkOVRl7CURBbgLcyo3ou1rS1E),
[MH_05](https://drive.google.com/open?id=10l5x4rXDy-wT6dKArQJj-ZIgtJYPPf4a),
[V1_01](https://drive.google.com/open?id=12eV1sR9CxSfsY1_aMoP8Qkh0Yu2LLzOB),
[V1_02](https://drive.google.com/open?id=1OX0VAU1GpTqSyEdissCDn-447Q1qZkU4),
[V1_03](https://drive.google.com/open?id=10KSoatF7z4tOZfW0zyPawJkMURMV7bKY),
[V2_01](https://drive.google.com/open?id=1RBZlWS4enKmH5hmfIGx-8AaWcrvYMXRB),
[V2_02](https://drive.google.com/open?id=1ZzuP_BqMGF86YlbKtUtz3M3W7v8WDSHr),
[V2_03](https://drive.google.com/open?id=1g8oymW8y0dN8cJlwUj-sBx4uYp9jKVIp)

Results
<details>

<summary>srrg2_proslam</summary>
<br>

[Latest benchmarks (open-loop) on KITTI][kitti]

| | | | |
|:-:|:-:|:-:|:-:|
|KITTI 00 path|KITTI 00 errors|KITTI 01 path|KITTI 01 errors|
|<img width="300" height="300" src="results/kitti/00/00.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="150" src="results/kitti/00/00_tl.png"><br><img width="300" height="150" src="results/kitti/00/00_rl.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="300" src="results/kitti/01/01.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="150" src="results/kitti/01/01_tl.png"><br><img width="300" height="150" src="results/kitti/01/01_rl.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|
|KITTI 02 path|KITTI 02 errors|KITTI 03 path|KITTI 03 errors|
|<img width="300" height="300" src="results/kitti/02/02.png">|<img width="300" height="150" src="results/kitti/02/02_tl.png"><br><img width="300" height="150" src="results/kitti/02/02_rl.png">|<img width="300" height="300" src="results/kitti/03/03.png">|<img width="300" height="150" src="results/kitti/03/03_tl.png"><br><img width="300" height="150" src="results/kitti/03/03_rl.png">|
|KITTI 04 path|KITTI 04 errors|KITTI 05 path|KITTI 05 errors|
|<img width="300" height="300" src="results/kitti/04/04.png">|<img width="300" height="150" src="results/kitti/04/04_tl.png"><br><img width="300" height="150" src="results/kitti/04/04_rl.png">|<img width="300" height="300" src="results/kitti/05/05.png">|<img width="300" height="150" src="results/kitti/05/05_tl.png"><br><img width="300" height="150" src="results/kitti/05/05_rl.png">|
|KITTI 06 path|KITTI 06 errors|KITTI 07 path|KITTI 07 errors|
|<img width="300" height="300" src="results/kitti/06/06.png">|<img width="300" height="150" src="results/kitti/06/06_tl.png"><br><img width="300" height="150" src="results/kitti/06/06_rl.png">|<img width="300" height="300" src="results/kitti/07/07.png">|<img width="300" height="150" src="results/kitti/07/07_tl.png"><br><img width="300" height="150" src="results/kitti/07/07_rl.png">|
|KITTI 08 path|KITTI 08 errors|KITTI 09 path|KITTI 09 errors|
|<img width="300" height="300" src="results/kitti/08/08.png">|<img width="300" height="150" src="results/kitti/08/08_tl.png"><br><img width="300" height="150" src="results/kitti/08/08_rl.png">|<img width="300" height="300" src="results/kitti/09/09.png">|<img width="300" height="150" src="results/kitti/09/09_tl.png"><br><img width="300" height="150" src="results/kitti/09/09_rl.png">|
|KITTI 10 path|KITTI 10 errors|
|<img width="300" height="300" src="results/kitti/10/10.png">|<img width="300" height="150" src="results/kitti/10/10_tl.png"><br><img width="300" height="150" src="results/kitti/10/10_rl.png">|

[Latest benchmarks (open-loop) on Malaga][malaga]

| | | | |
|:-:|:-:|:-:|:-:|
|Malaga 01 path|Malaga 01 errors|Malaga 02 path|Malaga 02 errors|
|<img width="300" height="300" src="results/malaga/01/01.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="150" src="results/malaga/01/01_tl.png"><br><img width="300" height="150" src="results/malaga/01/01_rl.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="300" src="results/malaga/02/02.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="150" src="results/malaga/02/02_tl.png"><br><img width="300" height="150" src="results/malaga/02/02_rl.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|
|Malaga 03 path|Malaga 03 errors|Malaga 04 path|Malaga 04 errors|
|<img width="300" height="300" src="results/malaga/03/03.png">|<img width="300" height="150" src="results/malaga/03/03_tl.png"><br><img width="300" height="150" src="results/malaga/03/03_rl.png">|<img width="300" height="300" src="results/malaga/04/04.png">|<img width="300" height="150" src="results/malaga/04/04_tl.png"><br><img width="300" height="150" src="results/malaga/04/04_rl.png">|
|Malaga 05 path|Malaga 05 errors|Malaga 06 path|Malaga 06 errors|
|<img width="300" height="300" src="results/malaga/05/05.png">|<img width="300" height="150" src="results/malaga/05/05_tl.png"><br><img width="300" height="150" src="results/kitti/05/05_rl.png">|<img width="300" height="300" src="results/malaga/06/06.png">|<img width="300" height="150" src="results/malaga/06/06_tl.png"><br><img width="300" height="150" src="results/kitti/06/06_rl.png">|
|Malaga 07 path|Malaga 07 errors|Malaga 08 path|Malaga 08 errors|
|<img width="300" height="300" src="results/malaga/07/07.png">|<img width="300" height="150" src="results/malaga/07/07_tl.png"><br><img width="300" height="150" src="results/malaga/07/07_rl.png">|<img width="300" height="300" src="results/malaga/08/08.png">|<img width="300" height="150" src="results/malaga/08/08_tl.png"><br><img width="300" height="150" src="results/malaga/08/08_rl.png">|
|Malaga 09 path|Malaga 09 errors|Malaga 10 path|Malaga 10 errors|
|<img width="300" height="300" src="results/malaga/09/09.png">|<img width="300" height="150" src="results/malaga/09/09_tl.png"><br><img width="300" height="150" src="results/malaga/09/09_rl.png">|<img width="300" height="300" src="results/malaga/10/10.png">|<img width="300" height="150" src="results/malaga/10/10_tl.png"><br><img width="300" height="150" src="results/malaga/10/10_rl.png">|
|Malaga 11 path|Malaga 11 errors|Malaga 12 path|Malaga 12 errors|
|<img width="300" height="300" src="results/malaga/11/11.png">|<img width="300" height="150" src="results/malaga/11/11_tl.png"><br><img width="300" height="150" src="results/malaga/11/11_rl.png">|<img width="300" height="300" src="results/malaga/12/12.png">|<img width="300" height="150" src="results/malaga/12/12_tl.png"><br><img width="300" height="150" src="results/malaga/12/12_rl.png">|

[Latest benchmarks (open-loop) on EuRoC][euroc]

| | | | |
| :-: | :-: | :-: | :-: |
| mh_01 [ATE: <b>0.x00</b> m] | mh_02 [ATE: <b>0.x00</b> m] | mh_03 [ATE: <b>0.x00</b> m] | mh_04 [ATE: <b>0.x00</b> m] |
|<img width="300" height="225" src="results/euroc/mh_01/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/euroc/mh_02/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/euroc/mh_03/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/euroc/mh_04/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|
| mh_05 [ATE: <b>0.x00</b> m] | v1_01 [ATE: <b>0.x00</b> m] | v1_02 [ATE: <b>0.x00</b> m] | v1_03 [ATE: <b>0.x00</b> m] |
|<img width="300" height="225" src="results/euroc/mh_05/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/euroc/v1_01/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/euroc/v1_02/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/euroc/v1_03/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|
| v2_01 [ATE: <b>0.x00</b> m] | v2_02 [ATE: <b>0.x00</b> m] | v2_03 [ATE: <b>0.x00</b> m] |
|<img width="300" height="225" src="results/euroc/v2_01/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/euroc/v2_02/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/euroc/v2_03/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|

</details>

#### 3D RGBD SLAM
Datasets (json)
 - ICL Living Room [0](https://drive.google.com/open?id=1HBEmz0qBxFUTrk1K4pJIUNPpJYhgMHYD),
[1](https://drive.google.com/open?id=1kojNwhrWbdK3nR08M2vIUv66wW-NZHo1),
[2](https://drive.google.com/open?id=1lJW4CcPiZmPOwJtMJR2kdfKc2sWtsJ_F),
[3](https://drive.google.com/open?id=17CaZbEGhBUMFwxTJSv71yl6VNtMF7i9k),
Office Room [0](https://drive.google.com/open?id=11_Ms0a9RVgjwEVQ88CzTzP_DOR7ZpbgR),
[1](https://drive.google.com/open?id=156ULtiSmisW42kccWE43kjr55qkj7gbt),
[2](https://drive.google.com/open?id=1Qnx4En3SnfaUCOfiiyu5DFpBywkoLdp7),
[3](https://drive.google.com/open?id=1ENF8aKAi9z92T8Res0vlY7O-icBLUU_F)
 - TUM [fr1_room](https://drive.google.com/open?id=1qPavF3iHuoeG7P_cVUUQ_nyXKE0miR-z),
[fr2_desk](https://drive.google.com/open?id=1oE9VPUYcu5XLzFI15XnuyO-d6Ok_EttC),
[fr2_large_with_loop](https://drive.google.com/open?id=16T5ObpdfqTjVah7WE1aAtQiRLOwPOyMd),
[fr3_long](https://drive.google.com/open?id=16zdgsLWRiyLrDXokm9dp6w7I6ZE5Gqqv)

Results
<details>

<summary>srrg2_proslam</summary>
<br>

[Latest benchmarks (open-loop) on ICL][icl]

| | | | |
| :-: | :-: | :-: | :-: |
| icl_living_room_0 [ATE: <b>0.005</b> m] | icl_living_room_1 [ATE: <b>0.00x</b> m] | icl_living_room_2 [ATE: <b>0.00x</b> m] | icl_living_room_3 [ATE: <b>0.00x</b> m] |
|<img width="300" height="225" src="results/icl/living_room_0/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/icl/living_room_1/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/icl/living_room_2/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/icl/living_room_3/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|
| icl_office_room_0 [ATE: <b>0.00x</b> m] | icl_office_room_1 [ATE: <b>0.00x</b> m] | icl_office_room_2 [ATE: <b>0.00x</b> m] | icl_office_room_3 [ATE: <b>0.00x</b> m] |
|<img width="300" height="225" src="results/icl/office_room_0/trajectory_error.png">|<img width="300" height="225" src="results/icl/office_room_1/trajectory_error.png">|<img width="300" height="225" src="results/icl/office_room_2/trajectory_error.png">|<img width="300" height="225" src="results/icl/office_room_3/trajectory_error.png">|

[Latest benchmarks (open-loop) on TUM][tum]

| | | | |
| :-: | :-: | :-: | :-: |
| fr1_room [ATE: <b>0.00x</b> m] | fr2_desk [ATE: <b>0.00x</b> m] | fr2_large_with_loop [ATE: <b>0.00x</b> m] | fr3_long [ATE: <b>0.063</b> m] |
|<img width="300" height="225" src="results/tum/fr1_room/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/tum/fr2_desk/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/tum/fr2_large_with_loop/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|<img width="300" height="225" src="results/tum/fr3_long/trajectory_error.png">&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;|

</details>

[kitti]: https://gitlab.com/srrg-software/srrg2_proslam/commit/3c6058a807b203a26ded47bc434e2e4b57f14f4c
[malaga]: https://gitlab.com/srrg-software/srrg2_proslam/commit/c707793a24da4d5add38e9a30d8c070280851695
[icl]: https://gitlab.com/srrg-software/srrg2_proslam/commit/86f46304efcc6f26d1df80223a4df3b27ecba52c
[tum]: https://gitlab.com/srrg-software/srrg2_proslam/commit/86f46304efcc6f26d1df80223a4df3b27ecba52c
[euroc]: https://gitlab.com/srrg-software/srrg2_proslam/commit/5eee90f16ef54c18ad88571f57b997c4dd197299
