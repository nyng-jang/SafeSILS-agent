# SafeSILS-Agent

ISO 26262 기능안전 검증의 반복 작업(테스트 케이스 작성, 고장 주입, 로그 분석, 추적성 매트릭스)을 LLM 에이전트가 자율 수행하는 SILS 검증 파이프라인. 검증 대상은 AEB(자동 긴급 제동)다.

> HARA 등급, FTTI, TTC 임계값 등 모든 수치는 학습용 가정값이며 실제 양산 AEB 값이 아니다.

## 구조

```
requirements/aeb_safety.yaml   HZ → SG → FSR → TSR 요구사항 (추적성의 단일 원천)
safesils/
  scenario.py    시나리오(JSON) = 테스트 케이스, 고장 정의
  plant.py       자차·선행차 종방향 모델, 레이더·카메라 물체 센서, 제동 액추에이터
  bus.py         가상 CAN: E2E 보호(CRC, alive counter)와 버스 고장 주입
  controller.py  AEB 제어기(검증 대상)와 안전 메커니즘
  sim.py         1 ms 주기 SILS 루프, ControllerUnderTest 인터페이스
  judge.py       요구사항별 규칙 기반 PASS/FAIL 판정
  generator.py   기준·경계값·복합 고장 시나리오 생성
  store.py       SQLite 결과 저장(runs, verdicts, signals)
  external.py    외부 제어기 인터페이스: Simulink FMU 어댑터, 원시 CAN 프레임 슬롯
  b2b.py         백투백 비교 (Python ↔ MATLAB 코드 개루프, Python ↔ FMU 폐루프)
  cli.py         명령줄 도구
matlab/
  aeb_controller_step.m  AEB 제어기 MATLAB 구현 (codegen 호환, Python과 동일 동작)
  build_aeb_model.m      Simulink 모델(AEB_Controller.slx) 생성과 FMU 내보내기
  replay_inputs.m        기록된 입력 개루프 재생 (MATLAB, Octave 공용)
  b2b_simulink.m         MATLAB 안에서 Simulink 모델과 Python 기준 출력 백투백 비교
  b2b_vectors/           백투백 입력·기대 출력 벡터 (CSV)
scenarios/baseline/  요구사항별 회귀 시나리오 17개
tests/               회귀 테스트와 판정기 변이(mutation) 테스트
```

## 빠른 시작

```bash
pip install -e ".[dev]"
pytest -q
safesils run scenarios/baseline --db out/results.db
safesils generate sweep --req TSR-02 --n 7 --out out/sweep
safesils generate combo --n 50 --seed 1 --out out/exploratory
safesils run out/sweep out/exploratory --db out/results.db --no-signals
```

## Simulink 제어기 (FMU)

검증 대상 제어기는 Simulink 모델로도 존재한다. `matlab/aeb_controller_step.m`이 제어 로직이고, `build_aeb_model.m`이 이를 MATLAB Function 블록에 넣은 10 ms 고정 스텝 모델을 만든다.

```matlab
% MATLAB R2026b, 저장소 루트에서
cd matlab
build_aeb_model(true)   % AEB_Controller.slx 저장, AEB_Controller.fmu (FMI 2.0 CS) 내보내기
```

```bash
pip install -e ".[fmu]"
safesils b2b scenarios/baseline --fmu matlab/AEB_Controller.fmu    # Python 기준 제어기와 폐루프 비교
safesils run scenarios/baseline --fmu matlab/AEB_Controller.fmu    # FMU 제어기로 전체 판정
```

Python을 쓸 수 없는 PC(예: Windows 스마트 앱 컨트롤이 pip 패키지 DLL을 차단하는 경우)에서는 MATLAB 안에서 백투백 비교를 한다. `matlab/b2b_vectors`에 기준 시나리오 17개의 제어기 입력과 Python 기준 출력이 들어 있다.

```matlab
cd matlab
b2b_simulink   % AEB_Controller.slx를 같은 입력으로 개루프 시뮬레이션하고 모든 출력 비교
```

제어기를 바꾸면 `safesils b2b scenarios/baseline --export matlab/b2b_vectors`로 벡터를 다시 만든다(테스트가 최신 여부를 검사한다).

MATLAB 없이도 `safesils b2b scenarios/baseline`이 GNU Octave로 MATLAB 코드를 개루프 재생해 Python 기준 제어기와 모든 활성화의 모든 출력을 비교한다. FMU 어댑터는 `tests/fmu_stub`의 대체 FMU로 테스트한다.

제어기와 하네스 사이의 인터페이스는 실제 ECU가 보는 신호다. 센서마다 활성화당 최대 4개의 원시 CAN 프레임 `[present counter crc obj status_ok range_raw vel_raw]`(거리 0.01 m, 상대속도 0.01 m/s, CRC-8 SAE J1850)과 차속, 운전자 입력, 고장 주입 신호를 받는다.

## 안전 메커니즘과 요구사항

| 메커니즘 | 요구사항 | 판정 규칙 |
| --- | --- | --- |
| E2E 검사(CRC, alive counter) | TSR-01 | 손상 프레임 도착 후 10 ms 안에 `e2e_*` |
| 메시지 타임아웃 | TSR-02 | 유효 프레임 없이 150 ms + 1주기 안에 `timeout_*` |
| 레이더·카메라 교차 검증 | FSR-01, TSR-03 | 두 센서가 같은 물체를 확인할 때만 제동, 불일치 100 ms 지속 시 제동 금지 |
| 독립 제동 모니터 | TSR-04 | 비정상 제동 명령 즉시 차단, 50 ms 지속 시 FAIL_SAFE |
| FAIL_SAFE 출력 게이트 | TSR-05 | FAIL_SAFE 후 50 ms 안에 제동 요청 0 |
| 운전자 오버라이드 | TSR-06 | 개입 후 100 ms 안에 제동 해제 |
| 단일 센서 고장 DEGRADED | TSR-07 | 고장 후 200 ms 안에 DEGRADED와 고장 램프 |
| 태스크 워치독 | TSR-08 | 주기 위반 3회 연속 시 FAIL_SAFE |
| 안전 목표 | SG-01 | 위험 없는 1.5 m/s² 초과 감속이 FTTI 200 ms 이하 |
| 안전 목표 | SG-03 | 충돌 회피, 또는 충돌 전 운전자 경고(FCW·고장 램프) |

SG-01, TSR-02, TSR-05는 모든 실행에서 불변 조건으로 검사한다.

## 설계 원칙

- **판정은 결정적이다.** PASS/FAIL은 `judge.py`의 규칙이 내리고, LLM 에이전트는 시나리오 제안과 원인 추론만 맡는다(ISO 26262-8 도구 신뢰도 관점).
- **판정기는 제어기 내부를 믿지 않는다.** 센서 값과 프레임 수신 여부는 시뮬레이터가 기록한 실측값(ground truth)으로 판단한다.
- **제어기는 교체 가능하다.** `sim.ControllerUnderTest`만 만족하면 Simulink에서 FMU로 내보낸 제어기도 같은 시나리오와 판정기로 검증할 수 있다. FMU 제어기가 내보내야 할 출력은 `judge.py` 상단에 정리되어 있다.
- **판정기도 검증한다.** 변이 테스트가 안전 메커니즘을 하나씩 약화시켜 판정기가 FAIL을 내는지 확인한다.

## 추적성 ID

`HZ-nn` → `SG-nn` → `FSR-nn` → `TSR-nn` → `TC-<요구사항>-nnn` → `RUN-<날짜>-<순번>`. 복합 고장 TC(`TC-EXP-*`)는 A-SPICE SYS.5의 탐색적 검증으로 `exploratory: true`가 붙는다.

## 로드맵

1. ISO 26262 분석과 AEB 안전 개념 ✓
2. Python SILS, 시나리오 생성기, 규칙 기반 판정기 ✓
3. Simulink AEB 제어기와 FMU 연동, Python 제어기와 백투백 비교 ✓ (FMU 내보내기는 MATLAB 환경에서 실행)
4. LangGraph 에이전트: 자연어 명령, 엣지 케이스 탐색, 로그 분석·원인 추론
5. 추적성 매트릭스·검증 보고서 자동 생성, GitHub Actions 회귀 검증
