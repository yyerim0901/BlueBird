## IoT제어 프로젝트

* 시뮬레이터 및 프로젝트 관련 파일 다운로드
  - https://drive.google.com/drive/folders/1rp54qL31ZIoHet7A9BlvpoDCCdGVsvLK?usp=sharing

(위 경로에 위치한 프로그램 및 문서는 SSAFY 과정 내에서만 사용할 수 있으며 무단 복제 및 반출, 배포를 금합니다.)





**Req 1 ROS 메시지 통신 노드 실행**

| publisher<br />![image-20210830153945555](./images/image-20210830153945555.png) |
| ------------------------------------------------------------ |
| **subscriber**<br />![image-20210830153926246](./images/image-20210830153926246.png)                                            **** |
| **rqt result**<br />![req1_3](./images/req1_3.PNG)           |
| **auto mode에서 자동으로 터틀 봇 제어**<br />![req1_5_autodrive](./images/req1_5_autodrive.PNG) |





**Req 2 IoT(로봇, 가전, 환경)의 상태, 제어 메시지 송수신**

| **시뮬레이션화면**<br />![image-20210830200723439](./images/image-20210830200723439.png) |
| ------------------------------------------------------------ |
| **앞으로 전진**<br />![image-20210831195510458](./images/image-20210831195510458.png)<br /> |
| **회전**<br />![image-20210831195644838](./images/image-20210831195644838.png) |
| **Moving in the auto mode**<br />![req1_4](./images/req1_4.png) |
| **가전 기기 제어 - all on **<br />![req2_1](./images/req2_1.png) |
| ![req2_allon](./images/req2_allon.PNG)                       |
| **가전 기기 제어 - all off**<br />![req2_1](./images/req2_2.png) |
| ![req2_alloff](./images/req2_alloff.PNG)                     |
| **환경 메시지 수신**<br />![req2_weather](./images/req2_weather.PNG) |



**Req 3 카메라 데이터 수신 및 영상처리**

gray scaling
size scaling: h = 1/2 * h, w =  1/2 * w

| **이미지 변환 후 화면**<br />![req3](./images/req3.PNG) |
| ------------------------------------------------------- |





**Req 4 Hand Control 제어 메시지 송신**

| 들어 올릴 객체 생성<br />![req4_makeobject](./images/req4_makeobject.PNG) |
| ------------------------------------------------------------ |
| **0번 입력 시**<br />![req4_0](./images/req4_0.PNG)          |
| **물체에 다가가서 들 수 있는 상태로 0 입력시**<br />![req4_0_canlift](./images/req4_0_canlift.PNG) |
| **2번 입력으로 물건 들기**<br />![req4_1](./images/req4_1.PNG) |
| **1번 입력으로 내려 놓을 수 있는지 확인 - 못내려 놓는 경우** <br />![req4_2_cannotput](./images/req4_2_cannotput.PNG) |
| **1번 입력으로 내려 놓을 수 있는지 확인 - 내려 놓을 수 있는 경우** <br />![req4_2_canput](./images/req4_2_canput.PNG) |
| **3번 입력으로 내려 놓기 **<br />![req4_3](./images/req4_3.PNG) |



**Req 5-1 주행 기록계(Odometry를 이용한 위치 추정 및 기록)**

| **로봇 위치 추정 테스트**<br />![req5_1](./images/req5_1.PNG) |
| ------------------------------------------------------------ |
| **테스트 결과**<br />![req5_2](./images/req5_2.png)          |
| **한 바퀴 돌아서 제자리 온 경우 rviz2 화면**<br />![req5_3](./images/req5_3.PNG) |



**Req 5-2 상대경로 생성**

| **상대 경로 생성 - 텍스트파일로 경로 기록**<br />![req5_6make_path_file](./images/req5_6make_path_file.PNG)<br />**상대 경로 생성 - rviz에서 global path로 확인**<br />![req5_1_global_path](./images/req5_1_global_path.PNG) |
| ------------------------------------------------------------ |
| **Test - 이전 위치와 현재위치의 거리 계산**<br />![req5_4make_path](./images/req5_4make_path.PNG)<br />![req5_5make_path_aws](./images/req5_5make_path_aws.PNG) |

**Req 5-3 경로 읽어오기 및 경로 추종** 

| **Local path가 잘 적용되는지 확인**<br />![req5_1_local_path](./images/req5_1_local_path.PNG)<br />**이동 시 local path가 잘 적용되는 것 확인**<br />![req5_1_local_path_move](./images/req5_1_local_path_move.PNG) |
| ------------------------------------------------------------ |
| **Auto mode에서 자동으로 이동 확인**<br />![req5_ing](./images/req5_ing.PNG)<br />auto mode에서 속도가 나고 global path에서 저장해 놓았던 경로로 이동하는 것 확인<br /><br />**도착 확인**<br />![req5_finish](./images/req5_finish.PNG) |


