
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
https://github.com/user-attachments/assets/e3aba4df-99dc-4d68-8e0c-35457fd25711

  

**PATROL (자동순찰) 모드 주행**  
https://github.com/user-attachments/assets/bea7e5ff-7d94-4662-b653-a62f1b6e1600

  
---

## 개발 환경
<img width="1085" height="579" alt="image" src="https://github.com/user-attachments/assets/42971f83-c278-4d60-8501-e2ded8bb589e" />

---
## 사용 모듈  및 핀맵 하드웨어 연결  
<img width="622" height="567" alt="image" src="https://github.com/user-attachments/assets/67e79759-0c27-4777-ad06-a5fc3c9e06c4" />
<img width="2512" height="1812" alt="image" src="https://github.com/user-attachments/assets/cdfae1f4-966f-4b64-9d80-7627686f2fdd" />
<img width="644" height="586" alt="image" src="https://github.com/user-attachments/assets/9539791a-2bcb-46d6-b169-1ecf866402ed" />
<img width="1696" height="1242" alt="image" src="https://github.com/user-attachments/assets/cc91a75f-8b24-4bfd-93d1-1abe728dac77" />
<img width="1786" height="1461" alt="image" src="https://github.com/user-attachments/assets/1db90c7b-38bc-4202-93b7-6f643953590a" />

 

---
## 소프트웨어 블락도  

- 슈퍼루프(Non-Blocking): HAL_Delay() 최소화, 다중 작업을 이벤트/시간 기반 틱으로 번갈아 처리
- 시간·이벤트 기반: HAL_GetTick()으로 스케줄
- BT 수신은 인터럽트로 받고 루프에서 처리
- 상태 틱 구조: patrol / cruise / melody / safety /arm 등  함수(tick) 들을 매 루프 갱신
- 주기 예시: DHT11 10초 / 초음파 60 ms (좌·우 교대)
- 전체 코드 및 주석은 첨부파일 main.c와 smartcar.h를 참조

<br>
<img width="1529" height="2190" alt="image" src="https://github.com/user-attachments/assets/4c3b8ad7-90f9-41f3-868e-1b583e8555ae" />









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

https://github.com/user-attachments/assets/305a502c-8d28-4acd-9f63-e025dc545eda  

  


https://github.com/user-attachments/assets/91ee5a69-d2fd-44d5-b6e6-96c3da15ccdb


</details>









