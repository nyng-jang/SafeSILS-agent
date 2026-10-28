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
  cli.py         명령줄 도구
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
3. Simulink/Stateflow AEB 제어기와 FMU 연동, Python 제어기와 백투백 비교
4. LangGraph 에이전트: 자연어 명령, 엣지 케이스 탐색, 로그 분석·원인 추론
5. 추적성 매트릭스·검증 보고서 자동 생성, GitHub Actions 회귀 검증
