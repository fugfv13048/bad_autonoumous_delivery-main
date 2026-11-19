"""Receive input on the ROS graph and publish the results of the fleet tick."""
# ─────────────────────────────────────────────────────────────
# [이 파일은 무엇을 하나요?]
# 이 파일은 "ROS 2 노드"입니다. 노드 = ROS 2 세계에서 돌아가는 작은 프로그램 1개.
# 하는 일은 3단계로 요약됩니다.
#   1) 입력 받기  : UI, 침대이탈 감지(Part5) 등이 보내는 메시지를 받는다.
#   2) 판단 맡기  : 받은 메시지를 "두뇌"(FleetCore)에 넘겨서 로봇 배차/제어를 계산시킨다.
#   3) 결과 내보내기: 두뇌가 계산한 명령/상태를 로봇과 UI에 다시 발행(publish)한다.
# 즉, 이 파일 자체는 "ROS 통신 담당(우체부)"이고, 실제 판단은 core/fleet_core.py가 합니다.
#
# [ROS 2 용어 먼저]
#  - topic(토픽)   : 우편함 이름. 누군가 글을 올리고(publish), 구독자(subscribe)가 읽는다.
#  - publisher     : 토픽에 메시지를 올리는 사람.
#  - subscription  : 토픽의 메시지가 올라올 때마다 호출되는 함수 등록.
#  - service(서비스): "요청 -> 응답" 1회성 통화. (토픽은 일방적 방송, 서비스는 전화)
#  - timer(타이머) : 일정 간격으로 함수를 자동 호출.
# ─────────────────────────────────────────────────────────────

# ── import: 다른 파일/라이브러리에서 도구를 가져오는 문법 ──
# "from A import B" = A라는 곳에서 B라는 도구만 쏙 가져온다.

# dataclass(데이터 보관용 클래스)의 값을 "일부만 바꾼 복사본"을 만드는 함수.
# 원본은 건드리지 않고 새 객체를 돌려준다. (아래에서 설정을 덮어쓸 때 사용)
from dataclasses import replace
# partial: 함수에 인자를 미리 "고정"해 새 함수를 만드는 도구. (아래에서 자세히 설명)
from functools import partial
# Path: 파일 경로를 문자열이 아닌 "객체"로 안전하게 다루는 도구. (/ 기호로 경로를 이을 수 있음)
from pathlib import Path

# 설치된 ROS 패키지의 share(설정파일 등이 설치되는) 폴더 위치를 알려주는 함수.
from ament_index_python.packages import get_package_share_directory
# rclpy: ROS 2의 파이썬 라이브러리. 거의 모든 ROS 파이썬 코드가 가져온다.
import rclpy
# 외부에서 "종료해라"라고 신호가 왔을 때 발생하는 예외(에러의 한 종류).
from rclpy.executors import ExternalShutdownException
# Node: 모든 ROS 노드의 "설계도 원본". 우리 클래스가 이걸 물려받는다.
from rclpy.node import Node
# 메시지 타입: Bool은 True/False, String은 문자열 하나를 담는 표준 메시지.
from std_msgs.msg import Bool, String
# Trigger: "요청 내용 없이 실행만 시키고, 성공 여부+메시지를 돌려받는" 표준 서비스 타입.
from std_srvs.srv import Trigger

# 앞의 점(.)은 "이 파일과 같은 패키지 안에서"라는 뜻의 상대 경로 import.
from .core.config import load_config      # 설정 파일(yaml)을 읽어 설정 객체로 만드는 함수
from .core.fleet_core import FleetCore    # 배차/판단을 담당하는 "두뇌" 클래스
from .core.schemas import dumps           # 파이썬 데이터를 JSON 문자열로 바꾸는 함수
from .robot_link import RobotLink         # 로봇 1대와의 통신을 담당하는 클래스


# def = 함수 정의. 괄호 안은 받는 값(인자)인데 여기선 없음.
def default_config():
    # get_package_share_directory("패키지명") -> 그 패키지의 설치 폴더 경로
    # Path(...) / "config" / "fleet.yaml" : 폴더 뒤에 하위 경로를 이어붙임
    # str(...) : 경로 객체를 일반 문자열로 변환
    # 결과: 기본 설정 파일(fleet.yaml)의 전체 경로를 돌려준다(return).
    return str(Path(get_package_share_directory("part3_fleet_manager")) / "config" / "fleet.yaml")


# class 클래스이름(부모클래스): 부모의 기능을 모두 물려받는 새 설계도를 만든다 (상속).
# 즉 FleetManagerNode는 "Node의 모든 기능 + 우리가 추가한 기능"을 가진다.
class FleetManagerNode(Node):
    # __init__ : 객체가 "처음 만들어질 때" 자동 실행되는 준비 함수(생성자).
    # self = "지금 만들어지는 나 자신". 클래스 안 함수는 첫 인자로 항상 self를 받는다.
    def __init__(self):
        # super() = 부모(Node). 부모의 __init__을 먼저 실행해 노드 이름을 "fleet_manager"로 등록.
        super().__init__("fleet_manager")

        # declare_parameter(이름, 기본값): 실행할 때 바깥에서 바꿀 수 있는 설정값을 선언.
        # 예) ros2 run ... --ros-args -p config_file:=다른파일.yaml
        # .value : 실제 값을 꺼낸다. 바깥에서 안 정하면 기본값(default_config())이 쓰인다.
        config = load_config(self.declare_parameter("config_file", default_config()).value)

        # 배차 방식(mode). 파일(config)에 적힌 값이 기본값이고, 실행 시 덮어쓸 수 있다.
        mode = self.declare_parameter("assignment_mode", config.assignment.mode).value
        # "not in" : 목록 안에 없으면 True. 허용되지 않은 값이면 에러를 일부러 발생시킨다.
        # raise = 에러를 던져 프로그램을 멈춘다. (잘못된 설정으로 위험하게 돌아가는 것 방지)
        if mode not in ("IDLE_BEST_BATTERY", "ROLE"):
            raise ValueError("assignment_mode must be IDLE_BEST_BATTERY or ROLE")

        # 침대 이탈 감시 관련 설정 묶음을 변수에 담아 둔다.
        monitor = config.safety.bed_monitor
        # "시작할 때 침대 감시가 꼭 있어야 하는가?" 설정. 이것도 바깥에서 바꿀 수 있다.
        required = self.declare_parameter("bed_monitor_require_at_start", monitor.require_at_start).value

        # replace(원본, 필드=새값): 원본은 그대로 두고 일부만 바꾼 "새 복사본"을 만든다.
        # 아래는 안쪽부터 바깥쪽으로 차례로 바꿔 끼우는 구조(양파 껍질처럼)입니다.
        #   1) bed_monitor의 require_at_start 교체
        #   2) 그걸 넣은 safety 교체
        #   3) 배차 mode를 넣은 assignment 교체
        # 마지막에 config 전체를 새 값으로 갱신한다. (줄 끝의 \ 없이도 괄호 안이라 줄바꿈 가능)
        config = replace(config, assignment=replace(config.assignment, mode=mode),
                         safety=replace(config.safety, bed_monitor=replace(monitor, require_at_start=required)))

        # DB(기록 저장소) 파일 경로 파라미터. 기본값 ""는 "지정 안 함"이라는 뜻.
        db_path = self.declare_parameter("db_path", "").value

        # 한 줄짜리 if문(삼항 연산): "A if 조건 else B"
        #   db_path가 비어있지 않으면(문자열은 비면 False) -> 경로를 반영한 새 config 사용
        #   비어있으면 -> 원래 config 사용
        # FleetCore(...) : 두뇌 객체를 만들어 self.core에 저장. (self.xxx에 넣어야 다른 함수에서도 쓴다)
        self.core = FleetCore(replace(config, db_path=db_path) if db_path else config)

        # 딕셔너리 컴프리헨션: {키: 값 for ... in ...} 반복문으로 딕셔너리를 한 번에 만든다.
        # config.robots는 {로봇이름: 로봇정보} 딕셔너리, .items()는 (이름, 정보) 쌍을 하나씩 꺼낸다.
        # 결과: 로봇마다 RobotLink(통신 담당)를 하나씩 만들어 {로봇이름: RobotLink} 로 저장.
        # 넘기는 값: self(이 노드), r(로봇이름), unit.ns(그 로봇의 네임스페이스),
        #            설정, self.core.enqueue(입력 대기열에 넣는 함수), self.now(현재시각 함수), 장애물 설정
        # (self.core.enqueue처럼 괄호 없이 쓰면 "함수 자체"를 건네는 것. 괄호를 붙이면 "실행"이 됩니다.)
        self.links = {r: RobotLink(self, r, unit.ns, config.robot_link, self.core.enqueue, self.now, config.obstacle)
                      for r, unit in config.robots.items()}

        # 출력용 publisher 4개를 딕셔너리로 만든다. {토픽이름: publisher}
        # create_publisher(메시지타입, 토픽이름, 큐크기): 큐크기 100 = 밀릴 때 최대 100개까지 보관.
        # 4개 토픽 모두 String(JSON 문자열)을 보낸다.
        #   /fleet/state        : 전체 로봇 현재 상태
        #   /fleet/decision_req : 사람(관제사)에게 "결정해 주세요" 요청 카드
        #   /mission/event      : 임무 중 발생한 사건/경고
        #   /ui/ack             : UI 요청에 대한 "접수했어요" 응답
        self.outputs = {name: self.create_publisher(String, name, 100) for name in
                        ("/fleet/state", "/fleet/decision_req", "/mission/event", "/ui/ack")}

        # 입력용 구독(subscription) 등록. UI가 보내는 문자열 토픽 3개를 반복문으로 한 번에 등록.
        #   /ui/mission_req  : 임무 요청
        #   /ui/operator_cmd : 관제사 명령
        #   /ui/decision     : 관제사의 결정 응답
        # create_subscription(타입, 토픽, 콜백함수, 큐크기): 메시지가 오면 "콜백함수(msg)"가 호출된다.
        # 문제: 콜백은 msg 하나만 받는데, 우리는 "어느 토픽에서 왔는지"도 알고 싶다.
        # 해결: partial(self.receive, topic) = receive 함수의 첫 인자(topic)를 미리 채워둔 새 함수.
        #       나중에 msg가 오면 self.receive(topic, msg)처럼 실행된다.
        for topic in ("/ui/mission_req", "/ui/operator_cmd", "/ui/decision"):
            self.create_subscription(String, topic, partial(self.receive, topic), 10)
        # Part5(침대이탈 감지)가 보내는 True/False 메시지 구독. (True = 이탈 감지)
        self.create_subscription(Bool, "/patient/bed_exit", partial(self.receive, "/patient/bed_exit"), 10)

        # 서비스 2개 등록: 일시정지 / 재개. 호출되면 self.trigger(service, request, response)가 실행된다.
        for service in ("/robot/pause", "/robot/resume"):
            self.create_service(Trigger, service, partial(self.trigger, service))

        # 타이머: 일정 주기마다 self.tick을 자동 호출. 주기(초) = 1 / 초당횟수(Hz).
        # 예) tick_hz가 10이면 0.1초마다 tick 실행. 이 tick이 이 노드의 "심장 박동"입니다.
        self.create_timer(1.0 / config.publish.tick_hz, self.tick)

    # 현재 시각을 "초(second)" 단위 실수로 돌려주는 도우미 함수.
    def now(self):
        # ROS 시계의 현재 시각은 나노초(10억분의 1초)로 나온다. 1e9(=10억)로 나눠 초로 변환.
        return self.get_clock().now().nanoseconds / 1e9

    # 구독한 토픽에 메시지가 올 때마다 실행되는 콜백.
    # topic은 partial로 미리 채워진 값, msg는 ROS가 넣어주는 실제 메시지.
    def receive(self, topic, msg):
        # msg.data = 메시지 속 실제 내용(문자열 또는 True/False).
        # 바로 처리하지 않고 두뇌의 "대기열(queue)"에 (토픽, 내용, 도착시각)으로 쌓아둔다.
        # -> 실제 처리는 아래 tick에서 한꺼번에 질서 있게 한다.
        self.core.enqueue(topic, msg.data, self.now())

    # 서비스(/robot/pause 등)가 호출될 때 실행. request는 이 서비스에선 내용이 없다.
    # response는 "빈 답장 용지"로, 여기에 결과를 채워 돌려줘야 한다.
    def trigger(self, service, request, response):
        # 두뇌의 service()가 (성공여부, 메시지) 두 값을 돌려준다.
        # "a, b = (x, y)" 문법 = 튜플 언패킹: 두 값을 한 번에 각각의 변수/필드에 넣는다.
        response.success, response.message = self.core.service(service, self.now())
        return response  # 채운 답장을 호출한 쪽에 돌려준다.

    # 파이썬 딕셔너리/리스트(payload)를 JSON 문자열로 바꿔 지정 토픽에 발행하는 도우미.
    def publish_json(self, topic, payload):
        # self.outputs[topic] : 위에서 만든 publisher를 토픽 이름으로 찾는다.
        # String(data=...)   : 문자열을 ROS 메시지 객체로 포장. dumps()가 JSON 문자열로 변환.
        # .publish(...)      : 실제 전송.
        self.outputs[topic].publish(String(data=dumps(payload)))

    # 타이머가 주기적으로 호출하는 "한 턴" 처리 함수. 이 노드의 핵심입니다.
    def tick(self):
        # 두뇌에게 "지금 시각 기준으로 한 턴 계산해줘" -> 결과 묶음(out)을 받는다.
        # out 안에는 commands, inspections, events, cards, acks, state, storage_error가 들어있다.
        out = self.core.tick(self.now())

        # 1) 로봇에게 보낼 이동 명령들. for문으로 하나씩 꺼내 해당 로봇의 통신 담당에게 전달.
        #    self.links[로봇이름]으로 알맞은 RobotLink를 찾아 .send(명령) 호출.
        for motion in out.commands:
            self.links[motion.robot].send(motion.command)

        # 2) 점검(inspection)이 필요한 로봇들에게 점검 요청.
        for robot in out.inspections:
            self.links[robot].request_inspection()

        # 3) 임무 이벤트들을 /mission/event 로 발행.
        for event in out.events:
            self.publish_json("/mission/event", event)
            # event는 딕셔너리. event["type"]처럼 [키]로 값을 꺼낸다.
            # 경고(WARNING)라면 터미널 로그에도 사유를 찍어 눈에 띄게 한다.
            if event["type"] == "WARNING":
                self.get_logger().warning(event["detail"]["reason"])

        # 4) 관제사에게 보여줄 "결정 요청 카드"들을 발행.
        for card in out.cards:
            self.publish_json("/fleet/decision_req", card)

        # 5) UI 요청에 대한 접수 응답(ack)들을 발행.
        for ack in out.acks:
            self.publish_json("/ui/ack", ack)

        # 6) 전체 상태. None이면 "이번 턴엔 보낼 상태 없음"이라는 뜻이므로 None이 아닐 때만 발행.
        #    (is not None : 값이 비어있음(None)이 아닌지 검사. 0이나 빈 값과 구분하려고 ==이 아닌 is를 쓴다.)
        if out.state is not None:
            self.publish_json("/fleet/state", out.state)

        # 7) 저장소(DB) 오류 문자열이 있으면(빈 문자열/None은 False) 에러 로그 출력.
        #    throttle_duration_sec=1.0 : 같은 로그가 1초에 최대 1번만 나오게 제한(로그 폭주 방지).
        if out.storage_error:
            self.get_logger().error(out.storage_error, throttle_duration_sec=1.0)

    # 노드가 종료될 때 호출되는 함수. 부모의 같은 이름 함수를 "덮어쓰기(오버라이드)" 했다.
    def destroy_node(self):
        # 종료 전에 DB 파일을 안전하게 닫아 데이터가 깨지지 않게 한다.
        self.core.store.close()
        # 그 다음 부모(Node)의 원래 종료 처리도 이어서 실행하고, 그 결과를 돌려준다.
        return super().destroy_node()


# 프로그램의 "출발점" 함수. (setup.py에서 이 함수를 실행 명령으로 연결해 둔다)
# args=None : 인자를 안 주면 기본값 None. ROS 실행 옵션을 받을 수 있게 열어둔 것.
def main(args=None):
    rclpy.init(args=args)          # ROS 2 통신 시스템을 시작 (노드를 만들기 전에 반드시 먼저)
    node = FleetManagerNode()      # 위에서 정의한 설계도로 실제 노드(객체)를 생성

    # try / except / finally : 에러 처리 문법
    #   try     : 일단 실행해 본다.
    #   except  : 지정한 에러가 나면 여기서 처리한다.
    #   finally : 에러가 나든 안 나든 "무조건" 마지막에 실행한다. (뒷정리용)
    try:
        # spin = 노드를 계속 살려두며 콜백(구독/타이머/서비스)을 처리하는 무한 반복.
        # Ctrl+C를 누르거나 종료 신호가 올 때까지 여기서 멈춰 있는다.
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C(KeyboardInterrupt) 또는 외부 종료 신호는 "정상 종료"이므로
        # pass(= 아무것도 안 함)로 조용히 넘어간다. 에러 메시지가 뜨지 않게 하려는 것.
        pass
    finally:
        node.destroy_node()        # 노드 정리 (위에서 덮어쓴 destroy_node가 실행되어 DB도 닫힘)
        rclpy.try_shutdown()       # ROS 통신 종료 (이미 종료됐어도 에러 없이 넘어가는 안전한 버전)
