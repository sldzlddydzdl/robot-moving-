# robot-moving-
협동로봇을 이용하여 특정 모션 수행하기

docker 를 사용하였습니다. 


- 원그리기
https://user-images.githubusercontent.com/71972556/149606096-c1f60e5b-cd18-415a-bb00-a232cecb4ca9.mp4









- Pick & Place

https://user-images.githubusercontent.com/71972556/149606364-00c3740e-98bc-4bc3-b64d-b281bff86aef.mp4


1. Docker 란 무엇인가?

도커는 컨테이너 기반의 오픈소스 가상화 플랫폼입니다.

서버에서 이야기하는 컨테이너는 다양한 프로그램, 실행환경을 컨테이너로 추상화하고 동일한 인터페이스를 제공하여 프로그램의 배포 및 관리를 단수화 해줍니다. 백엔드 프로그램, 데이터베이스 서버, 메시지 큐 등 어떤 프로그램도 컨테이너로 추상화할 수 있고 조립 PC, 
AWS, Azure, Google cloud 등 어디에서든 실행할 수 있습니다.

컨테이너는 격리된 공간에서 프로세스가 동작하는 기술입니다. 가상화 기술의 하나지만 기존 방식과는 차이가 있습니다. 기존의 가상화 방식은 주로 OS를 가상화하였습니다.

우리에게 익숙한 VMware 나 VirtulaBox 같은 가상머신은 호스트 OS위에 게스트 OS 전체를 가상화하여 사용하는 방식입니다. 이 방식은 여러 가지 OS 를 가상화 ( 리눅스에서 윈도우를 돌린다던가 ) 할 수 있고 비교적 사용이 간단하지만 무겁고 느려서 운영환경에선 사용할 수 없었습니다.

이러한 상황을 개선하기 위해 CPU의 가상화 기술(HVM)을 이용한 KVM(Kernel-based Virtual Machine) 과 반 가상화 (Paravirtualization) 방식의 Xen 이 등장합니다.

도커에서 가장 중요한 개념은 컨테이너와 함께 이미지라는 개념입니다.
이미지는 컨테이너 실행에 필요한 파일과 설정값들을 포함하고 있는 것으로 상태값을 가지지 않고 변하지 않습니다. 컨테이너는 이미지를 실행할 상태라고 볼 수 있고 추가되거나 변하는 값은 컨테이너에 저장됩니다. 같은 이미지에서 여러 개의 컨테이너를 생성할 수 있고 컨테이너의 상태가 바뀌거나 컨테이너가 삭제되더라도 이미지는 변하지 않고 그대로 남아 있습니다.

말그대로 이미지는 컨테이너를 실행하기 위한 모든 정보를 가지고 있끼 때문에 더 이상 의존성 파일을 컴파일하고 이것저것 설치할 필요가 없습니다. 이제 새로운 서버가 추가되면 미리 만들어 놓은 이미지를 다운받고 컨테이너를 생성만 하면 됩니다. 한 서버에 여러 개의 컨테이너를 실행할 수 있고 , 수십, 수백, 수천대의 서버도 문제없습니다.

도커 이미지는 Docker hub 에 등록하거나 Docker Registry 저장소를 직접 만들어 관리할 수 있습니다. 현재 공개된 도커 이미지는 50만개가 넘고 Docker hub의 이미지 다운로드 수는 80억회에 이릅니다. 누구나 쉽게 이미지를 만들고 배포할 수 있습니다.



2. ROS 이란 무엇인가?

로봇 운영체제(ROS,Robot Operating System)는 로봇 응용 프로그램을 개발할 때 필요한 하드웨어 추상화, 하위 디바이스 제어, 일반적으로 사용되는 기능의 구현, 프로세스간의 메시지 패싱, 패키지 관리, 개발환경에 필요한 라이브러리와 다양한 개발 및 디버깅 도구를 제공한다. ROS는 로봇 응용 프로그램 개발을 위한 운영체제와 같은 로봇 플랫폼이다. 하드웨어 플랫폼을 하드웨어 추상화로 포함하고 있으며, 로봇 응용 소프트웨어 개발을 지원을 위한 소프트웨어 플랫폼이면서 이기종의 하드웨어에서 사용 가능한 운영 체제와 같은 기능을 갖추고 있다.
쉽게 말하자면 로봇운영시스템이라고는 하지만 OS가 아닌 로봇과 로봇 사이의 통신을 만들어주거나, 로봇과 센서의 통신을 이어주거나, 사람들의 코드(오픈소스)와 쉽게 연결(통신)시켜주는 도구하고 생각하면 된다.

ROS 가 필요한 이유는 기존 로봇 개발 방식이 한계점이 있기 때문이다. 기존 로봇 개발 방식은 하드웨어 설계, 제어부터 제어기 , 비전, 네비게이션 등 모든 것을 개발해야 함
API 마다의 interface 가 다르고, 적용하는데 학습이 필요하다. 하드웨어에 의존적인 소프트웨어적 성격 때문에 로봇이 달라지면 소프트웨어 또한 수정이 필요하다. 소프트웨어를 작성하는데 하드웨어에 대한 지식이 필요하고, 디버깅을 하기 위해 디버깅 틀을 작성하거나 디버깅 API 코드를 삽입해야한다. OS에 의존적이어서 변경이 어렵거나 불가능함. 멀티PC, 로봇을 구성한는 경우 통신구축, 검증에 많은 시간을 소비해야 한다. 이러한 이유가 기존 로봇 개발 방식이 개발에 힘들다는 점이다.

로봇 소프트웨어 플랫폼이 있음으로 하드웨어 인터페이스를 통합하고, 
하드웨어를 추상화-규격화-모듈화 이 구조를 유지할수 있다. 가격은 내려가고 성능은 높일 수 있다. 하드웨어, 미들웨어(OS) , 소프트웨어로 제대로 분리가 가능해진다. 분업을 할수 있게된다. 사용자 수요에 맞는 서비스에 집중 할 수 있게된다. 

ROS 의 장점은 무료이고 사용자가 로봇 소프트웨어 플랫폼중에서 제일많고, 활성화된 커뮤니티 이고, 오픈소스가 많이있고, 기존 프로그램을 ROS 에 이식하기 쉽다.
하지만 단점으로는 단일 로봇을 위한 솔루션이고, 높은 PC의 연산 능력을 요구하고, 네트워크에 의존성이 높고, 보장되지 않는 실시간성, 임베디드 시스템에서 사용하기 어렵고, 리눅스 위주의 개발이다. 

ROS 의 특징은 
Node 간 메시지 교환 방식으로 프로그램을 잘게 나누어 공동으로 개발이 가능해진다.
명령어 도구, 시각화 도구, GUI 도구, 시뮬레이션 도구 등 이러한 다양한 도구들을 제공해준다. 모델링, 센싱, 인식, 네비게이션, 매니퓰레이션 등 기존의 로봇 개발에서는 이러한 제로베이스 부분인것도 다 개발해야하지만 ROS 는 이러한 기초적인 것은 기능을 지원해준다.



3. Docker 와 ROS 와 

Docker + ROS를 쓸 때 고려해야하는 점
•  Docker를 통해 개발을 할 때는 원격 코드 접근이 필요하다
o  이는 SSH나 VSCode의 Docker extension을 사용해서 디버깅을 할 수 있다.
•  ROS를 사용할 때 GUI로 결과를 보는 경우가 많다
o  VNC, X11, WayLand 등으로 시각화된 결과물을 보여줄 수 있다.
•  rosrun 기반으로 돌릴 때는 여러 터미널을 열 수 있어야한다.
o  tmux, terminator 등으로 하나의 컨테이너에 여러 bash 터미널을 접근해야한다.

기본적으로 각각의 컨테이너는 서로 통신이 불가능함.
하지만 도커의 네트워크 설정을 통해 각 컨테이너간의 통신이 가능하도록 할 수 있음.
도커의 네트워크 타입은 bridge, host, none가 있으며, 도커를 설치하게 되면 bridge 타입의 docker0 가상의 bridge network가 자동으로 생성됨.

sudo docker network ls 
Ifconfig 로 도커 네트워크에 대한 작업과 네트워크를 확인을 할 수 있다.

도커 컨테이너가 생성되면 두 가지 네트워크가 함께 생성됨.
•  eth0 : 컨테이너의 네트워크
•  veth : virtual ethernet의 약자로, 컨테이너의 네트워크와 Host PC의 가상 네트워크를 연결해주는 가상 네트워크

veth을 통해 컨테이너의 eth0 네트워크로부터 docker0 virtual bridge network 로 연결된다.


![image](https://user-images.githubusercontent.com/71972556/149606406-5916d403-b4e8-436e-bf6d-632c5f7d3cba.png)
그림 [ 1- 1 ]
네트워크 인터페이스 구조는 위와 같다.

그림 [ 1 - 1 ] 에서 
Brdige-network : 각각의 컨테이너들의 네트워크 통신이 가능하도록 하는 역할을 해준다.
Ros-master container : roscore 를 실행함으로써, ros master 역할을 하는 컨테이너이다.
Dockursim container : ursim 을 실행하는 컨테이너, ursim 내부 설정을 통해 ros 통신이 가능해진다.
Ur-ros-driver container : ur 로봇을 ros 를 통해 제어할 수 있도록 하는 driver 가 작동하는 컨테이너이다.

UR Robot 의 제어기는 기본적으로 ROS 통신을 지원하지 않는다.
따라서 Remote PC 의 UR ROS Driver 와 통신을 하기 위해서는 UR Robot 제어기에 추가 설정이 필요하다. UR Robot 제어기는 urcap 프로그램을 이용하여 여러 가지 기능을 추가적으로 구현할 수 있도록 구성되어있다. External Control 은 UR Robot 제어기에서 ROS 통신이 가능하도록 지원해주는 프로그램이며, URCAP 을 이용하여 설치를 수행할 수 있다.

Docker Compose 로 다른 사용자도 프로젝트에 참여하기 쉽게 해준다. 이는 YAML 파일에서 애플리케이션 스택을 정의하고 프로젝트 Repository root 파일에 저장하면 된다.
YAML 파일에 프로젝트에 대한 모든 설정을 적어놓고 Docker Compose 로 compose up 만 해주면 일반 로봇 개발 에서는 일일이 모든 것에대한 설정을 명령어로 다 하나씩 체크하면서 실행해야하는 것을 한곳에 모든 작업을 미리해두고 한번에 돌리는 거라 훨씬 편하다.


