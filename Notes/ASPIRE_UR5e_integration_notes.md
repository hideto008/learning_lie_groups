# ASPIREへのUR5e + Robotiq + RGB-D環境接続メモ

作成日: 2026-11-10  
対象: [NVlabs/ASPIRE](https://github.com/NVlabs/ASPIRE)  
目的: 独自のMuJoCo環境（UR5e + Robotiq 2F-85 + 天井固定RGB-Dカメラ）をASPIREへ接続し、ClaudeによるPick & Placeプログラムの生成・反復改善を行う。

> **注意**: このメモは手元のASPIREソースコードの検索結果・抜粋を基に整理したもの。クラス定義全体や実際のAPIシグネチャ、CLI設定の全体は未確認。以下の「実装例」は説明用であり、そのまま動く完成コードではない。

## 1. ASPIREで何を自動化するのか

- **人間側で準備**: MuJoCoのロボット・ハンド・カメラ・ワーク・パレット、ASPIREへの接続、タスク仕様、成功判定。
- **Claude Codeに開発依頼できる**: 環境モデル・Adapter・Registry・Evaluatorなどの作成支援。ただし、これはASPIREの反復タスク解決とは区別する。
- **ASPIRE + Claudeの反復ループ**: タスクを解くPythonプログラムを生成 → Harnessで実行 → ログと成功判定を確認 → プログラムを修正。

`run_program()` 自体はLLMを呼び出さない。**すでに存在するPythonプログラムを実行・評価する関数**であり、Claudeによる生成・修正はその外側のワークフロー。

## 2. 今回のタスク

- ロボット: UR5e
- グリッパ: Robotiq 2F-85
- カメラ: 天井固定RealSense相当のRGB-D（まずはMuJoCo仮想カメラ）
- 環境: パレット1個、直方体ワーク1個
- 作業: RGB-Dでワーク位置・姿勢を推定し、把持してパレット中心へ配置
- 例示的成功条件: XY中心誤差5 mm以下、開放直前の底面ギャップ0〜1 mm、姿勢誤差5°以下、開放後に安定、禁止接触なし
- 安全制約: **実機には接続せず、URScriptも送信しない**。生成コード実行環境を実機ネットワークから隔離する。

成功条件の数値は初期案。実験仕様として確定するまでは勝手に緩和しない。

## 3. 全体の接続構造

```text
Claude Code（Python Skillを作成・修正）
      │
      ▼
program.py  ──────────────┐
                          ▼
run.py → ProgramExecutor(tools).execute_program(source)
                          │
                          ▼
                   CapApi / ToolRegistry
                     │           │
                     ▼           ▼
              EnvironmentAdapter  Providers
                     │           ├─ SAM3（セグメンテーション）
                     ▼           ├─ ContactGraspNet（把持候補）
              UR5e MuJoCo        ├─ PyRoKi（IK候補）
                                 └─ cuRobo（IK・計画候補）

実行後: adapter.check_success() → RunRecorderに結果保存
```

※実際の関数呼び出しはCapApi内で分岐する。上図は依存関係を表す概念図。

## 4. EnvironmentAdapterとは

定義場所:

```text
src/cap_harness/protocols.py:145
class EnvironmentAdapter(Protocol):
```

`Protocol` は構造的部分型のインターフェース。**自作の `UR5eAdapter` は `EnvironmentAdapter` を継承・importしなくてもよい**。必要なメソッド・属性が型定義と適合していれば、型チェッカー上はEnvironmentAdapterとして扱える。

`@runtime_checkable` が付いていれば `isinstance(adapter, EnvironmentAdapter)` の実行時検査が可能。ただし、これは基本的に属性・メソッドの存在確認であり、引数・戻り値の型や動作の正しさを保証しない。静的型検査と実行テストを併用する。

これまでの確認で挙がったメソッド・属性（**実装時は必ず `protocols.py` を原本として確認すること**）:

```python
@property
 def embodiment(self) -> str: ...

def reset(self, task_ref: object, seed: int) -> Observation: ...
def get_task_context(self) -> TaskContext: ...
def get_observation(self) -> Observation: ...
def get_robot_state(self) -> RobotState: ...
def step(self, action: RobotAction) -> StepResult: ...
def execute_trajectory(self, trajectory: Trajectory | SynchronizedTrajectory) -> ExecutionResult: ...
def set_gripper(self, position: float, *, arm: str = "primary") -> ExecutionResult: ...
def set_grippers(self, positions: Mapping[str, float]) -> ExecutionResult: ...
def close(self) -> None: ...
```

**補足**: `check_success()` は手元の `run.py` で直接呼ばれているが、先に確認した `EnvironmentAdapter(Protocol)` のメソッド一覧には含まれていなかった。したがって、現行の実行経路では実装が必要。Protocolと実行側の要求に差がある点に注意。

```python
# run.py 抜粋
 task_success = adapter.check_success()  # type: ignore[attr-defined]
```

Adapterは、ASPIREのデータ構造とMuJoCoの状態・操作を橋渡しする。たとえば `step(RobotAction)` を受け取り、アクチュエータ指令と `mujoco.mj_step()` を実行して `StepResult` を返す。単純な `data.ctrl[:6] = q` だけでは、アクチュエータの種類、関節順序、時間積分、収束判定を扱いきれない。

## 5. Adapterの実装例と登録箇所

既存実装:

```text
src/cap_harness/libero/adapter.py
src/cap_harness/robosuite/adapter.py
src/cap_harness/behavior/adapter.py
src/cap_harness/yam_real/adapter.py
```

自作の配置案:

```text
src/cap_harness/ur5e/
  __init__.py
  adapter.py
  registry.py
```

### 5.1 `run.py` のAdapter選択

手元で確認した `src/cap_harness/run.py` の `run_program()` は、`benchmark` に応じてRegistry・metadata・Adapterクラスを選択する。

```python
if benchmark == "libero-pro":
    registry = LiberoSuiteRegistry()
    metadata = registry.resolve(suite, task_id)
    adapter_type = LiberoAdapter
    extension_allowlist = None
elif benchmark == "behavior":
    ...
elif benchmark == "yam_real":
    ...
else:
    # robosuite
    ...
```

独自benchmarkを増やす場合の**概念的な変更例**:

```python
elif benchmark == "ur5e-pick-place":
    from .ur5e.adapter import UR5eAdapter
    from .ur5e.registry import UR5eTaskRegistry
    registry = UR5eTaskRegistry()
    metadata = registry.resolve(suite, task_id)
    adapter_type = UR5eAdapter
    extension_allowlist = None
```

同時に `run_program()` 冒頭の許可benchmark集合、CLI引数の許可値・ヘルプ、テストなども確認する。`run.py` の分岐追加だけでは自動的にCLIから使えるとは限らない。

### 5.2 Adapterの生成

`run.py` で確認した実際のコード:

```python
adapter_kwargs = {
    "registry": registry,
    "run_observer": recorder,
    "camera_width": camera_width,
    "camera_height": camera_height,
    "horizon": max_steps,
}
# yam_real は別経路
adapter = adapter_type(**adapter_kwargs)
adapter.reset(metadata, seed)
```

UR5eAdapterの `__init__()` はこれらを受け取るか、UR5e専用の生成分岐を作る必要がある。`metadata` はRegistryの `resolve(suite, task_id)` が返す。

### 5.3 CapApiへの接続

```python
api = CapApi(
    adapter,
    segmentation_provider=segmentation,
    grasp_providers={"contact-graspnet": contact_graspnet},
    ik_providers=ik_providers,
    trajectory_planning_providers={"curobo": curobo},
    integrated_pose_planning_providers={"curobo-integrated": curobo},
    default_camera="agentview",
    recorder=recorder,
)
```

UR5eのカメラ名が `overhead` なら `default_camera` も合わせる。Providerは名前を登録しただけでUR5e対応になるわけではなく、運動学モデル・関節次元・サービス側のロボット設定を別途確認する。

## 6. CapApi / ProgramExecutor / 成功判定

`src/cap_harness/api.py` の `CapApi.__init__`:

```python
def __init__(self, adapter: EnvironmentAdapter, *, ...):
    self._adapter = adapter
```

`CapApi.get_task_context()` の実例:

```python
context = self._adapter.get_task_context()
```

生成プログラムの公開APIとAdapterのメソッド名は**必ずしも1対1で一致しない**。`CapApi` がProviderやAdapterへ振り分ける。

`run.py` 後半の実行処理:

```python
tools = api.register_tools(
    ToolRegistry(public_extension_allowlist=extension_allowlist)
    if extension_allowlist is not None
    else ToolRegistry()
)
execution = ProgramExecutor(tools, recorder=recorder).execute_program(source)
task_success = adapter.check_success()
```

- `source` は `program_path` から読み込んだPythonソース。
- `register_tools()` は生成プログラムに利用可能な関数を登録する。
- `ProgramExecutor` はそのPythonを実行する。
- `check_success()` は環境の真値からタスク成功を判定する。
- `RunRecorder` はプログラム、ログ、結果、動画などを記録する。

タスク仕様MarkdownはClaudeに読ませる指示書であり、**それだけで機械的な成功判定にはならない**。成功条件はEvaluator/AdapterのPythonコードとして実装し、生成プログラムには採点用の真値を渡さない。

## 7. embodimentと関節次元

検索結果:

```text
src/cap_harness/contracts.py:14 DEFAULT_EMBODIMENT = "robosuite"
src/cap_harness/contracts.py:15 JOINT_DIMENSIONS = MappingProxyType(...)
src/cap_harness/api.py:108-110 Adapterのembodimentが未登録ならDEFAULT_EMBODIMENTへフォールバック
```

UR5eは6関節。`embodiment = "ur5e"` を使う場合は、`JOINT_DIMENSIONS` などへの登録が必要か確認する。実際の辞書内容は未確認。

特に注意すべき検索結果:

```text
src/cap_harness/providers/curobo/service.py:617
src/cap_harness/providers/curobo/service.py:747
joint_dimension = JOINT_DIMENSIONS[
    "yam_real" if request.model == "yam_real" else "robosuite"
]
```

この分岐では、UR5eを指定しても既存の `robosuite` 側へ流れる可能性がある。関節数の登録だけでなく、cuRoboのロボットモデル設定・変換処理まで確認すること。PyRoKiについても同様。

## 8. 指示書はMarkdownでよいか

**よい。ただしファイルを置くだけでASPIREが自動実行するとは限らない。** Claude Codeに対象ファイルと作業内容を明示する。

例: `projects/ur5e_pick_place/task.md`

```markdown
# UR5e Pick & Place

## 目的
RGB-Dカメラから直方体ワークの姿勢を推定し、把持してパレット中心へ配置する。

## 成功条件
- ワーク中心とパレット中心のXY誤差 <= 0.005 m
- グリッパ開放直前のワーク底面とパレット上面のギャップ: 0～0.001 m
- ワーク姿勢誤差 <= 5 deg
- 開放後にパレット上で安定
- 禁止衝突なし

## 制約
- MuJoCoのみ。実機制御、URScript送信、実機IPへの接続は禁止
- ワーク認識はRGB-D観測を使い、シミュレータ真値は評価にのみ使用
- 成功条件を自動変更しない
```

Claude Codeへの指示例:

```text
ASPIREの .claude/skills/iterative-debugging/SKILL.md と
 docs/api-reference.md を確認してください。
projects/ur5e_pick_place/task.md を読んでください。
まずUR5e環境・Adapter・Registry・Provider・Evaluatorの接続状態を点検し、
不足点と実装方針を報告してください。
環境が整った後、MuJoCoのみでPython Skillを生成・実行・反復改善してください。
成功条件の変更、実機接続、URScript送信は禁止します。
```

## 9. 次に会社PCで調べること

### A. Protocolの正確な契約

```bash
sed -n '135,215p' src/cap_harness/protocols.py
```

### B. Contractのデータ型と関節次元

```bash
sed -n '1,48p' src/cap_harness/contracts.py
rg -n 'class (Observation|RobotAction|RobotState|StepResult|ExecutionResult|TaskContext)' src/cap_harness/contracts.py
```

### C. 既存Adapterの初期化・実装例

```bash
sed -n '45,240p' src/cap_harness/robosuite/adapter.py
rg -n 'def (reset|step|execute_trajectory|set_gripper|check_success)' src/cap_harness/robosuite/adapter.py
```

### D. CLIからrun_programへ渡す経路

```bash
rg -n 'run_program\(|choices=.*robosuite|libero-pro' src/cap_harness
```

### E. UR5eのProvider対応

```bash
rg -n 'JOINT_DIMENSIONS|DEFAULT_EMBODIMENT' src/cap_harness
rg -n 'request.model|robot_state.embodiment' src/cap_harness/providers
```

### F. 公開APIの登録

```bash
rg -n 'def register_tools|def get_observation|def execute_trajectory|def set_gripper' src/cap_harness/api.py
```

## 10. 推奨する実装順序

1. `protocols.py` / `contracts.py` の要求を確認する。
2. UR5e + Robotiq + RGB-DのMuJoCo単体環境を作り、観測・関節制御・把持を検証する。
3. `UR5eAdapter` と `UR5eTaskRegistry` を実装し、既存の `run.py` に接続する。
4. **最初は単純な関節動作だけ**をASPIREのPythonプログラムから実行する。
5. カメラ観測、姿勢推定、UR5e用IK/軌道計画Providerを接続する。
6. 真値を使う独立した成功判定を実装する。
7. `task.md` をClaude Codeに読ませ、ASPIREの反復デバッグでPick & Placeを解かせる。
8. ワーク初期位置・姿勢をランダム化して再現性・頑健性を検証する。

## 11. 現時点での理解の要点

- **Protocol**: Adapterに要求するメソッド・属性の型契約。継承は必須ではない。
- **Adapter**: 共通データ構造とMuJoCo固有の制御・観測を変換。
- **Registry**: suite / task_idからタスク情報を取得。
- **CapApi**: 生成プログラムに見せる共通操作API。AdapterとProviderを利用。
- **Provider**: セグメンテーション、把持、IK、軌道計画などの処理。
- **ProgramExecutor**: 生成済みPythonソースを実行。
- **check_success**: 環境真値を使った成功判定。現行 `run.py` は直接呼ぶ。
- **run.py**: これらを生成・接続して1エピソードを実行するオーケストレータ。
- **Claude Code + ASPIRE**: Python Skillの生成と実行結果に基づく改善を繰り返す。

