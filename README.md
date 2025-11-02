
# SMARTCAR_PROJECT
---
## 프로그램 목표  

- 교육과정중 학습한 BT04-A, DHT11, ST7735S, SG90, HC-SR04, MH-FMD 등의 모듈을 이용하여 다기능 스마트카를 제작한다  

- 각 기능을 통합하며 비동기(Non-Blocking) 소프트웨어 제어 및 알고리즘 구현 능력 향상
  
- 각 모듈에 대한 정리
  
  https://github.com/OOminhOO/STM32/tree/main/NUCLEO_F103RB

---
## 구현 영상

**주행/로봇팔 MANUAL 모드 영상**  
https://github.com/OOminhOO/STM32_SMARTCAR_PROJECT/blob/main/INTEGRATE.mp4  

  
**CRUISE 모드 주행**  
 


https://github.com/user-attachments/assets/20852ed5-7ca6-4b6f-a671-449aedacbfeb


  

**PATROL (자동순찰) 모드 주행**  




https://github.com/user-attachments/assets/a51ae9a6-fc97-4047-9d60-1a12eeaefe8e



---
## 주요 모듈 및 핀맵 하드웨어 연결
<img width="500" height="450" alt="image" src="https://github.com/user-attachments/assets/67e79759-0c27-4777-ad06-a5fc3c9e06c4" />
<img width="500" height="470" alt="image" src="https://github.com/user-attachments/assets/9539791a-2bcb-46d6-b169-1ecf866402ed" />  
<img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/cc91a75f-8b24-4bfd-93d1-1abe728dac77" />
<img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/1db90c7b-38bc-4202-93b7-6f643953590a" />  


<img width="1367" height="702" alt="image" src="https://github.com/user-attachments/assets/ef05d406-4f71-44de-9663-725a0b41b977" />  

<img width="2512" height="1812" alt="image" src="https://github.com/user-attachments/assets/cdfae1f4-966f-4b64-9d80-7627686f2fdd" />




 

---
## 소프트웨어 블락도  

- 슈퍼루프(Non-Blocking): HAL_Delay() 최소화, 다중 작업을 이벤트/시간 기반 틱으로 번갈아 처리
- 시간·이벤트 기반: HAL_GetTick()으로 스케줄
- BT 수신은 인터럽트로 받고 루프에서 처리
- 상태 틱 구조: patrol / cruise / melody / safety /arm 등  함수(tick) 들을 매 루프 갱신
- 주기 예시: DHT11 10초 / 초음파 60 ms (좌·우 교대)
- 전체 코드 및 주석은 첨부파일 main.c와 smartcar.h를 참조

<br>
<img width="500" height="800" alt="image" src="https://github.com/user-attachments/assets/4c3b8ad7-90f9-41f3-868e-1b583e8555ae" />

---

## 스마트카 기능 설명 및 제어 방법  
<details>
<summary>펼치기/접기 </summary>  
  
스마트폰의 Serial Bluetooth Terminal (Android)앱을 통하여 stm32보드와 블루투스 통신을 통하여 스마트카를 제어한다.


### 주행모드  
#### Manual mode  
스마트폰 블루투스 입력을 통하여 제어  
세이프모드 on 상태일 때, 전진 상태일 때 초음파 센서에 20cm이하 감지시 경고음 2초 후진 2초 진행  
후진시 엘리제를 위하여 후진 알림음 
<br>  

w,s,a,d,f : 전진,후진,좌회전,우회전,정지  
u : 총돌방지 safe 모드 on/off 토글  
c : 크루즈 모드 on  
p : 자율 순찰 모드 on  
<br>  

#### 크루즈 모드  
센서 값 무효일때 또는 17cm 이하 일때  
->정지  
<br>  
  
17cm ~25 cm 사이 일 때  
-> 부드럽게 펄스 주행  
<br>  

25cm ~ 측정범위  
-> 연속 전진  
<br>
c : 크루즈 모드 off, 주행 manual mode 전환
<br>  
<br>  


#### 자율순찰 모드  

초음파 센서값 25cm이하일때  
-> 좌,우 거리값비교 -> 더 먼쪽으로 0.5초 회전  
<br>  
초음파 센서값이 양쪽다 15cm 이하일때  
  ->후진 0.5초  
<br>
p : 자율순찰 모드 off, 주행 manula mode 전환
<br>  
<br>  


### 로봇팔 모드 
w,s,a,d,f : 상,하,좌,우,정지  
u : 원위치  
x :  주행모드 전환  

</details>




---


## 프로젝트 발표 ppt 사진
<details>
<summary>펼치기/접기 </summary>

<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/f99004a6-ffff-458c-8ae8-05328ed5a075" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/a8dc09e9-4034-4705-bd9b-ae313947dcf8" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/afe5d1ca-d858-4328-86b8-79b887b8bb0b" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/42bb6002-36ff-4467-ac79-68601d7be7c7" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/74dc75c2-c3f1-4666-8431-2e3503d64f97" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/dcd04cb0-05fb-4cde-9216-edeb7899d4bd" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/df15c551-e8ea-4bce-a7ca-f70d4d61ec9a" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/c343d756-d05c-49ab-ac8c-71a44a36a864" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/c80573a6-ff8c-4995-8688-ba7a4978e4a2" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/12a8867c-bd5d-4566-9413-600e91dd3da8" />
<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/25a03fd2-cbe6-410e-8514-1c6d73ed29a8" />

<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/0b81fc6d-e8c4-4bfd-8c3e-0743e241fe7a" />

<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/0c34a9fb-54e5-4b93-95bb-0693c3fe72b2" />

<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/54586b4d-dd42-4ab4-8df6-55fa2153198e" />

<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/1b70e723-c28c-473a-b241-71154a1cd6ef" />

<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/7011b4c7-0b86-494f-a244-e50756da7f55" />


<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/f88f2d80-0140-41d2-94d2-6deff8116640" />


<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/de7a9aa2-e536-465d-8403-fd9ed9337b6c" />


<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/a30d76a8-07d7-4a60-a66d-c8da4aed29d5" />


<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/ff3a4d3b-958c-41ae-8277-50b64bf1f38e" />


<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/85bcc9ff-2180-4837-af8b-011e18d0af51" />


<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/1edc1013-d71e-4823-a75a-0091c377e598" />


<img width="3200" height="1800" alt="image" src="https://github.com/user-attachments/assets/5a03dea8-c330-4848-9dfa-0acd9f8788e0" />



  




</details>









