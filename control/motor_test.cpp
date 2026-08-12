// 지상 모터 테스트 도구. MAV_CMD_DO_MOTOR_TEST로 모터 1개를 골라 스로틀
// kStartThrottlePercent(5%)로 돌리기 시작하고, 키보드로 스로틀을 올리고
// 내릴 수 있게 한다.
//
// 안전 설계:
//   - param4(timeout)에 짧은 값을 넣어 반복 전송한다. ArduPilot은 그
//     시간 안에 다음 DO_MOTOR_TEST가 도착하지 않으면 스스로 모터를 멈춘다
//     (데드맨 스위치). 즉 이 프로그램이 죽거나 링크가 끊겨도 모터가 켜진
//     채로 남지 않는다.
//   - 시작 전 이미 armed 상태면 거부한다 (setting/play_bacchanale_motors.py
//     의 connect()와 같은 관례).
//   - 프로펠러 제거를 사람이 직접 타이핑해서 확인해야 시작한다.
//   - 정상 종료(q)/Ctrl+C(SIGINT)/예외 어느 경로든 스로틀 0 명령을 보내고
//     터미널 raw 모드를 복원한 뒤 빠져나간다.
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "drone_lib.hpp"

namespace {

constexpr double kCommandTimeoutSec = 1.0;    // FC가 이 시간 동안 갱신 없으면 자동 정지
constexpr double kResendIntervalSec = 0.3;    // kCommandTimeoutSec보다 충분히 짧게
constexpr double kStartThrottlePercent = 5.0;
constexpr double kDefaultStepPercent = 5.0;
constexpr double kMaxThrottlePercent = 100.0;
constexpr double kDefaultDwellSec = 3.0;      // 자동전개 모드에서 모터 1개당 도는 시간

// ArduPilot의 MAV_CMD_DO_MOTOR_TEST에는 "모터 N개 동시 회전" 파라미터가
// 없다 - 한 커맨드는 항상 모터 1개만 지정한다. 대신 ArduCopter 쪽 구현
// (Copter::mavlink_motor_test_start() -> AP_Motors::output_test_seq())은
// 커맨드가 올 때마다 "그 모터 채널"에만 PWM을 쓰고 다른 채널은 건드리지
// 않으며, 커맨드가 들어올 때마다 전체 모터-테스트 세션의 데드맨 타이머
// (param4)만 새로 갱신한다. 그래서 모터 1..motor_count에 대한
// DO_MOTOR_TEST를 아주 짧은 간격으로 계속 돌려가며 보내면, 각 채널이 마지막
// 값을 유지한 채 타이머만 계속 갱신되어 사실상 전부 동시에 도는 것처럼
// 동작한다 (simultaneous 모드, 아래 send_all_motors 참고). 이건 ArduCopter
// 구현 세부사항에 기대는 방식이라 펌웨어 버전에 따라 달라질 수 있다 - 동시에
// 안 도는 것처럼 보이면 즉시 중단할 것.

std::atomic<bool> g_stop_requested{false};

void handle_sigint(int) { g_stop_requested = true; }

// stdin을 non-canonical/no-echo raw 모드로 바꿔 Enter 없이 키 입력을 바로
// 읽는다. 소멸자에서 항상 원래 설정으로 복원 - 예외/Ctrl+C 종료 경로에서도
// 터미널이 raw 모드로 남는 것을 막는다.
class RawTerminal {
public:
    RawTerminal() {
        if (tcgetattr(STDIN_FILENO, &original_) != 0) {
            throw std::runtime_error("tcgetattr(stdin) 실패");
        }
        struct termios raw = original_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            throw std::runtime_error("tcsetattr(stdin) 실패");
        }
    }
    ~RawTerminal() { tcsetattr(STDIN_FILENO, TCSANOW, &original_); }

    RawTerminal(const RawTerminal&) = delete;
    RawTerminal& operator=(const RawTerminal&) = delete;

private:
    struct termios original_ {};
};

// 대기 중인 키가 없으면 -1.
int read_key_nonblocking() {
    unsigned char c;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    return n == 1 ? static_cast<int>(c) : -1;
}

std::string statustext_to_string(const mavlink_statustext_t& st) {
    return std::string(st.text, strnlen(st.text, sizeof(st.text)));
}

void send_motor_test(MavConnection& vehicle, int motor, double throttle_percent,
                      double timeout_sec) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, 0, &msg, vehicle.target_system(), vehicle.target_component(), MAV_CMD_DO_MOTOR_TEST,
        0,
        static_cast<float>(motor),                          // param1: 모터 번호 (1부터)
        static_cast<float>(MOTOR_TEST_THROTTLE_PERCENT),     // param2: 스로틀 타입
        static_cast<float>(throttle_percent),                // param3: 스로틀 값 (%)
        static_cast<float>(timeout_sec),                     // param4: 타임아웃 (데드맨 스위치)
        0,                                                    // param5: motor count (0=이 모터만)
        static_cast<float>(MOTOR_TEST_ORDER_DEFAULT),         // param6: 테스트 순서
        0);                                                   // param7: 미사용
    vehicle.send(msg);
}

// simultaneous 모드: 1..motor_count 전부에 대해 짧은 간격으로 연달아
// DO_MOTOR_TEST를 보낸다 - send_motor_test 위 주석 참고.
void send_all_motors(MavConnection& vehicle, int motor_count, double throttle_percent,
                      double timeout_sec) {
    for (int m = 1; m <= motor_count; ++m) {
        send_motor_test(vehicle, m, throttle_percent, timeout_sec);
    }
}

// 시작 전 안전 확인: 사람이 직접 "no prop"을 입력해야 진행된다. 영문 문구인
// 이유는 터미널/콘솔 환경에 따라 한글 입력이 가끔 먹지 않아서 확인 자체가
// 막히는 문제가 있었기 때문 - 대신 대소문자는 구분하지 않는다.
// 스킵 옵션 없음 - 실제 모터가 도는 동작이라 실수로 건너뛸 수 없게 한다.
void confirm_props_removed() {
    const std::string kPhrase = "no prop";
    std::cout << "\n=== 경고: 이 프로그램은 실제 모터를 회전시킵니다 ===\n"
              << "  - 프로펠러를 반드시 제거하고 진행하세요.\n"
              << "  - 기체를 단단히 고정하고, 사람/동물은 회전 반경 밖으로 물러나세요.\n"
              << "  - 동시 회전 모드('t' 또는 --together)는 모터 여러 개가 한꺼번에\n"
              << "    돌아 합산 추력이 커지니 고정을 더 단단히 하세요.\n"
              << "  - 계속하려면 아래에 정확히 입력하세요: \"" << kPhrase << "\"\n> " << std::flush;
    std::string input;
    std::getline(std::cin, input);
    auto to_lower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
        return s;
    };
    if (to_lower(input) != to_lower(kPhrase)) {
        throw std::runtime_error("확인 문구가 일치하지 않아 중단합니다.");
    }
}

struct Options {
    std::string address;
    int motor = 1;
    int motor_count = 4;
    double step_percent = kDefaultStepPercent;
    bool auto_cycle = false;
    double dwell_sec = kDefaultDwellSec;
    bool simultaneous = false;
};

Options parse_args(int argc, char** argv, const std::string& default_address) {
    Options opts;
    opts.address = default_address;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string(name) + " 값이 필요합니다");
            return argv[++i];
        };
        if (arg == "--address") {
            opts.address = next("--address");
        } else if (arg == "--motor") {
            opts.motor = std::stoi(next("--motor"));
        } else if (arg == "--motor-count") {
            opts.motor_count = std::stoi(next("--motor-count"));
        } else if (arg == "--step") {
            opts.step_percent = std::stod(next("--step"));
        } else if (arg == "--all") {
            opts.auto_cycle = true;
        } else if (arg == "--dwell") {
            opts.dwell_sec = std::stod(next("--dwell"));
        } else if (arg == "--together") {
            opts.simultaneous = true;
        } else {
            throw std::runtime_error("알 수 없는 옵션: " + arg);
        }
    }
    if (opts.motor_count < 1) throw std::runtime_error("--motor-count는 1 이상이어야 합니다");
    if (opts.motor < 1 || opts.motor > opts.motor_count) {
        throw std::runtime_error("--motor는 1.." + std::to_string(opts.motor_count) + " 범위여야 합니다");
    }
    if (opts.step_percent <= 0) throw std::runtime_error("--step은 0보다 커야 합니다");
    if (opts.dwell_sec <= 0) throw std::runtime_error("--dwell은 0보다 커야 합니다");
    if (opts.auto_cycle && opts.simultaneous) {
        throw std::runtime_error("--all과 --together는 동시에 줄 수 없습니다");
    }
    return opts;
}

void print_status(int motor, int motor_count, double throttle_percent, bool auto_cycle,
                   bool simultaneous) {
    std::cout << "\r";
    if (simultaneous) {
        std::cout << "[모터 1-" << motor_count << " 동시] 스로틀 " << throttle_percent << "%";
    } else {
        std::cout << "[모터 " << motor << "] 스로틀 " << throttle_percent << "%"
                   << (auto_cycle ? "  [자동전개 ON]" : "");
    }
    std::cout << "          " << std::flush;
}

}  // namespace

int run(int argc, char** argv) {
    YamlValue settings = drone::load_mavlink_settings();
    YamlValue ports = drone::load_port_settings();
    // 자체 포트(mavlink_control이 아님) - control.cpp가 "arm/비행하는 유일한
    // 프로세스"로 그 포트를 쓰고 있어서, 둘이 동시에 떠도 같은 UDP 포트를
    // 두고 bind 경합(패킷 가로채기)이 나지 않게 분리했다. mavlink_proxy.cpp가
    // 이 포트로도 fan-out해준다 - setting/port.yaml 참고.
    std::string default_address = with_port(settings["real"]["proxy_udp"]["address"].as_string(),
                                              ports.get_long_or("mavlink_motor_test", 14553));

    Options opts = parse_args(argc, argv, default_address);

    MavConnection& vehicle = drone::connect(opts.address);

    mavlink_heartbeat_t heartbeat{};
    if (!vehicle.wait_heartbeat(3.0, &heartbeat)) {
        throw std::runtime_error("현재 상태(armed 여부) 확인용 heartbeat를 받지 못했습니다.");
    }
    if (is_armed_from_heartbeat(heartbeat)) {
        throw std::runtime_error("기체가 이미 armed 상태입니다. 안전을 위해 모터 테스트를 거부합니다.");
    }

    confirm_props_removed();

    std::signal(SIGINT, handle_sigint);

    int motor = opts.motor;
    double throttle = kStartThrottlePercent;
    bool auto_cycle = opts.auto_cycle;
    bool simultaneous = opts.simultaneous;

    std::cout << "\n조작법: [w/+] 스로틀 증가  [s/-] 스로틀 감소  [0] 즉시 정지(0%)\n"
                 "        [1-" << opts.motor_count << "] 해당 모터로 즉시 전환  [n]/[p] 다음/이전 모터\n"
                 "        [a] 전체 모터 자동 순환 켜기/끄기 (모터당 " << opts.dwell_sec << "초, 순서대로)\n"
                 "        [t] 전체 모터 동시 회전 켜기/끄기 (--together 주석 참고, 펌웨어 의존적)\n"
                 "        [q] 종료 (모터 정지 후 종료)\n"
                 "모터 개수: " << opts.motor_count << ", 스텝: " << opts.step_percent << "%\n\n";

    RawTerminal raw_terminal;

    // 루프 안에서 무엇이 던져지든(전송 실패 등) 아래 stop_motor로 빠져
    // 정지 명령을 시도한 뒤 다시 던진다 - 모터가 돌고 있는 채로 프로그램만
    // 죽는 경로를 없앤다. 최종 안전망은 그래도 FC 쪽 데드맨 스위치
    // (kCommandTimeoutSec)다.
    std::exception_ptr pending_error;
    try {
        auto last_send = std::chrono::steady_clock::time_point::min();
        auto last_advance = std::chrono::steady_clock::now();
        bool running = true;
        while (running && !g_stop_requested) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_send >= std::chrono::duration<double>(kResendIntervalSec)) {
                if (simultaneous) {
                    send_all_motors(vehicle, opts.motor_count, throttle, kCommandTimeoutSec);
                } else {
                    send_motor_test(vehicle, motor, throttle, kCommandTimeoutSec);
                }
                last_send = now;
            }

            int key = read_key_nonblocking();
            bool changed = false;
            switch (key) {
                case 'w':
                case '+':
                case '=':
                    throttle = std::min(kMaxThrottlePercent, throttle + opts.step_percent);
                    changed = true;
                    break;
                case 's':
                case '-':
                    throttle = std::max(0.0, throttle - opts.step_percent);
                    changed = true;
                    break;
                case '0':
                    throttle = 0.0;
                    changed = true;
                    break;
                case 'n':
                case 'p': {
                    if (simultaneous) break;  // 동시 회전 중엔 개별 모터 선택 의미 없음
                    send_motor_test(vehicle, motor, 0.0, kCommandTimeoutSec);
                    motor = key == 'n' ? (motor % opts.motor_count) + 1
                                        : (motor == 1 ? opts.motor_count : motor - 1);
                    throttle = kStartThrottlePercent;
                    last_advance = now;
                    changed = true;
                    break;
                }
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9': {
                    if (simultaneous) break;  // 동시 회전 중엔 개별 모터 선택 의미 없음
                    int selected = key - '0';
                    if (selected <= opts.motor_count && selected != motor) {
                        send_motor_test(vehicle, motor, 0.0, kCommandTimeoutSec);
                        motor = selected;
                        throttle = kStartThrottlePercent;
                        last_advance = now;
                        changed = true;
                    }
                    break;
                }
                case 'a':
                    if (simultaneous) break;  // 먼저 't'로 동시 회전부터 꺼야 함
                    auto_cycle = !auto_cycle;
                    last_advance = now;
                    changed = true;
                    break;
                case 't': {
                    bool turning_on = !simultaneous;
                    if (turning_on) {
                        // 켜지는 순간: 순환 중이던 단일 모터를 먼저 멈추고,
                        // 전체를 시작 스로틀로 새로 시작한다.
                        send_motor_test(vehicle, motor, 0.0, kCommandTimeoutSec);
                        auto_cycle = false;
                        throttle = kStartThrottlePercent;
                    } else {
                        // 꺼지는 순간: 전체를 멈추고 단일 모터 모드로 복귀.
                        send_all_motors(vehicle, opts.motor_count, 0.0, kCommandTimeoutSec);
                    }
                    simultaneous = turning_on;
                    changed = true;
                    break;
                }
                case 'q':
                case 27:  // ESC
                    running = false;
                    break;
                default:
                    break;
            }

            // 자동전개: dwell_sec마다 다음 모터로 넘어간다 (simultaneous와는
            // 상호 배타적이라 auto_cycle이 true면 simultaneous는 항상 false).
            if (auto_cycle && now - last_advance >= std::chrono::duration<double>(opts.dwell_sec)) {
                send_motor_test(vehicle, motor, 0.0, kCommandTimeoutSec);
                motor = (motor % opts.motor_count) + 1;
                throttle = kStartThrottlePercent;
                last_advance = now;
                changed = true;
            }

            if (changed) {
                if (simultaneous) {
                    send_all_motors(vehicle, opts.motor_count, throttle, kCommandTimeoutSec);
                } else {
                    send_motor_test(vehicle, motor, throttle, kCommandTimeoutSec);
                }
                last_send = std::chrono::steady_clock::now();
                print_status(motor, opts.motor_count, throttle, auto_cycle, simultaneous);
            }

            mavlink_message_t msg;
            if (vehicle.recv_match({MAVLINK_MSG_ID_STATUSTEXT}, msg, 0.02)) {
                mavlink_statustext_t st;
                mavlink_msg_statustext_decode(&msg, &st);
                std::cout << "\nSTATUSTEXT: " << statustext_to_string(st) << std::endl;
                print_status(motor, opts.motor_count, throttle, auto_cycle, simultaneous);
            }
        }
    } catch (...) {
        pending_error = std::current_exception();
    }

    // 종료 경로(정상 q / Ctrl+C / 위 catch) 모두 1..motor_count 전체에 정지
    // 명령을 두 번 보낸다 - 현재 선택된 모터 하나만이 아니라 전체를 정리하는
    // 이유는, 이 세션 중 simultaneous 모드를 켰다 껐다 했다면 어느 채널이
    // 마지막으로 스로틀 값을 받았는지 여기서 장담할 수 없기 때문. UDP는
    // 유실될 수 있으므로 한 번으로는 부족하다. 이 전송 자체가 실패해도
    // (예: 연결이 이미 끊김) 무시하고 넘어간다 - 어차피 여기서 더 할 수
    // 있는 게 없고, FC 쪽 데드맨 스위치가 남은 안전망이다.
    try {
        for (int i = 0; i < 2; ++i) {
            send_all_motors(vehicle, opts.motor_count, 0.0, kCommandTimeoutSec);
            std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
        }
        std::cout << "\n모터 정지 명령 전송함." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "\n정지 명령 전송 실패(무시): " << e.what() << std::endl;
    }

    if (pending_error) std::rethrow_exception(pending_error);
    std::cout << "종료합니다." << std::endl;
    return 0;
}

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "\n" << e.what() << std::endl;
        return 1;
    }
}
