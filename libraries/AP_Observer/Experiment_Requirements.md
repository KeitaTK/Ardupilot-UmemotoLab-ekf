# 実験内容およびソフトウェア改修要件定義書

本資料は、論文の新規性（ペイロード・センサレスかつ機体側追加センサ不要のEKF外乱推定）を証明するために必要な**「実験計画」**と、それを実施するために現在のシステム（`AP_Observer`）に求められる**「機能改修の要件」**をまとめたものです。

---

## 1. 必要な実験計画（証明すべきシナリオ）

論文の論理展開を強力に裏付けるため、以下の3つの実験を実施します。

### 実験A: 未知周波数への自動収束（RLSの限界突破の証明）
*   **目的**: 荷物の紐の長さ（＝周波数）が未知であっても、EKFが自動で真の周波数を同定できることを示す。
*   **手順**:
    1. プログラム内の初期周波数を、実際の紐の長さから計算される理論周波数と**わざとずらした値**（例：実際の紐が1.0mなのに、初期設定を0.5m想定の周波数にする）に設定する。
    2. フライトを開始し、荷物に揺れを与える。
    3. EKFの推定周波数 $\omega$ が、時間経過とともに真の周波数へ収束していく様子をログからプロットする。
*   **取得データ**: 推定周波数 $\omega$ の時系列データ、実際の紐の長さから計算される理論周波数の値。

### 実験B: 突発的外乱へのロバスト性（Innovation Clippingの実用性証明）
*   **目的**: 実際の屋外環境で想定される突風や衝撃に対して、EKFが発散せずに安定して推定を継続できることを示す。
*   **手順**:
    1. 機体をホバリングさせ、制御をONにする。
    2. 扇風機で強い横風を当てる、あるいは棒で荷物を瞬間的に突く（インパルス外乱）。
    3. 強い外乱を受けた瞬間の「イノベーション（観測残差）」がクリッピングされ、推定器が破綻せずに即座に新しい振幅と位相を再推定する様子を示す。
*   **取得データ**: 観測外力 $z$、予測外力 $\hat{z}$、イノベーション $r$、および各軸の推定振幅 $A$。

### 実験C: 連続フライト内でのA/Bテスト（揺れ抑制効果の証明）
*   **目的**: 「制御なし（揺れが減衰しない）」状態から「制御あり（急速に減衰する）」状態への移行を1つのグラフで示し、制御の有効性を疑いの余地なく証明する。
*   **手順**:
    1. フライトを開始し、最初は**制御（補正ゲイン）をOFF**にしておく。
    2. 荷物を手で大きく揺らし、数秒間その揺れが持続（またはゆっくり減衰）するのを待つ。
    3. 飛行中の任意のタイミングで、プロポのスイッチにより**制御をON**にする。
    4. スイッチONの直後から、荷物の揺れ（エネルギー）が急速に減衰する様子をモーションキャプチャで計測する。
*   **取得データ**: 機体と荷物のMoCap位置データ（ここから振幅プロキシエネルギー $A(t)^2$ を算出）、制御ON/OFFのフラグ状態。

---

## 2. ソフトウェア改修要件定義

上記の実験を実機でスムーズに実施し、論文用のデータを取得するためには、現在の `AP_Observer` プログラムに対して以下の機能追加（要件）を満たす必要があります。

---

### 2.1 RCスイッチ割り当て設計（詳細決定版）

#### 2.1.1 使用可能チャンネルの全数調査

現在の実機パラメータ (`params_to_set`) における全RCチャンネルの使用状況:

| CH | 物理的種類 | パラメータ | 設定値 | 機能 | 操縦桿/スイッチ |
|----|-----------|-----------|--------|------|----------------|
| 1 | スティック（ジンバル） | (RCMAP固定) | - | ロール | 右スティック左右 |
| 2 | スティック（ジンバル） | (RCMAP固定) | - | ピッチ | 右スティック前後 |
| 3 | スティック（ジンバル） | (RCMAP固定) | - | スロットル | 左スティック前後 |
| 4 | スティック（ジンバル） | (RCMAP固定) | - | ヨー | 左スティック左右 |
| 5 | 3ポジションスイッチ | (FLTMODE_CH固定) | - | フライトモード切替 | プロポ左上 |
| 6 | **未使用（空き）** | `RC6_OPTION` | **0 (DO_NOTHING)** | — | プロポ右上？ |
| 7 | **未使用（空き）** | `RC7_OPTION` | **0 (DO_NOTHING)** | — | プロポ左下？ |
| 8 | **未使用（空き）** | `RC8_OPTION` | **0 (DO_NOTHING)** | — | プロポ右下？ |
| 9 | 2/3ポジションスイッチ | `RC9_OPTION` | 153 (ARMDISARM) | Arm/Disarm | ELRSレシーバー経由 |
| 10 | 2/3ポジションスイッチ | `RC10_OPTION` | 56 (LOITER) | Loiterモード切替 | プロポ |
| 11 | 2/3ポジションスイッチ | `RC11_OPTION` | 55 (GUIDED) | Guidedモード切替 | プロポ |
| 12 | **未使用（空き）** | `RC12_OPTION` | 0 (DO_NOTHING) | — | — |
| 13-16 | **未使用（空き）** | 未割り当て | 0 (DO_NOTHING) | — | — |

> **注**: CH5のフライトモード切替は、ArduPilotの `FLTMODE_CH` メカニズム（`Parameters.h` で定義）によってRCMAPにマッピングされる。`RC_OPTION`（AUX_FUNC）割り当てとは独立したメカニズムである。

#### 2.1.2 スイッチ割り当ての最終決定

**空きチャンネルの評価**:

| 候補 | 評価 | 採用判断 |
|------|------|----------|
| CH6 | プロポによってはスライダー/ノブの場合あり。スイッチ操作に不向きな可能性。 | ❌ 非推奨 |
| **CH7** | ほとんどのプロポで3ポジションスイッチ。物理的にアクセスしやすい。 | ✅ **採用: Observer 制御ON/OFF** |
| **CH8** | ほとんどのプロポで2ポジションまたはモーメンタリスイッチ。CH7と独立して操作可能。 | ✅ **採用: EKFリセット** |
| CH12 | プロポの奥側にあることが多く、飛行中の操作には不向き。 | ❌ 非推奨 |

**最終割り当て**:

| スイッチ | RCチャンネル | パラメータ | AUX_FUNC 値 | AUX_FUNC 名前 | スイッチ種別 | 機能 |
|----------|-------------|-----------|-------------|---------------|-------------|------|
| スイッチ1 | **CH7** | `RC7_OPTION = 47` | **47** | `USER_FUNC1` | 3ポジションスイッチ | Observer補正出力のON/OFF制御 |
| スイッチ2 | **CH8** | `RC8_OPTION = 48` | **48** | `USER_FUNC2` | モーメンタリスイッチ | EKF状態の再初期化（立ち下がりエッジ検出） |

#### 2.1.3 AUX_FUNC の選択根拠

**候補比較**:

| 選択肢 | AUX_FUNC値 | 長所 | 短所 | 判定 |
|--------|-----------|------|------|------|
| `USER_FUNC1/2` | 47, 48 | `UserCode.cpp` に空のハンドラが既存。コード変更が最小限。3値スイッチの `AuxSwitchPos` を引数で受け取れる。 | 名前が汎用的で意味が不明瞭（コードコメントで補う）。 | ✅ **採用** |
| 新規 `OBS_ENABLE` | 185（新設） | 意味が明確。`RC_Channel.h` の enum に名前が載る。 | enum 追加 + `do_aux_function()` へのハンドラ追加 or `AP_Observer` クラス側でのディスパッチ実装が必要。変更が ArduPilot コア（RC_Channel）に広がる。 | ❌ 変更範囲が大きすぎる |

**結論**: `USER_FUNC1 (47)` と `USER_FUNC2 (48)` を採用する。`UserCode.cpp` の `userhook_auxSwitch1/2` はこの目的のために設計されたフックであり、最小変更で要件を満たせる。

**動作フロー**:
```
プロポCH7 スイッチ操作
  → ELRS受信機 (CRSFプロトコル)
    → ArduPilot RC_Channel が PWM 値を読み取り AuxSwitchPos (LOW/MIDDLE/HIGH) に変換
      → RC_Channel::do_aux_function() が USER_FUNC1 (47) をディスパッチ
        → Copter::userhook_auxSwitch1(ch_flag) が呼び出される
          → AP_Observer::set_control_enabled(bool) を設定
```

---

### 2.2 干渉解析（ソフトウェア・電気・運用）

#### 2.2.1 ソフトウェア干渉解析

| # | 干渉項目 | 詳細 | 判定 |
|---|---------|------|------|
| 1 | **RCチャンネル独立性** | CH7, CH8 は別々のRCチャンネル。ArduPilotの `RC_Channel` クラスが各チャンネルを独立に管理。PWM値の取得から AUX_FUNC のディスパッチまでチャンネル間の依存関係なし。 | ✅ 干渉なし |
| 2 | **AUX_FUNC の重複** | `USER_FUNC1` (47), `USER_FUNC2` (48) は互いに異なる値。ArduPilot の AUX ディスパッチャ (`RC_Channel::do_aux_function()`) は `switch(ch_option)` で単一の分岐にディスパッチするため、同時に2つのハンドラが呼ばれることはない。 | ✅ 干渉なし |
| 3 | **ハンドラ関数の独立性** | `userhook_auxSwitch1()` と `userhook_auxSwitch2()` は独立した関数スコープ。staticローカル変数（CH8のエッジ検出用 `prev_ch8_state`）も関数ごとに分離。 | ✅ 干渉なし |
| 4 | **AP_Observer 内部状態の排他制御** | `_control_enabled` フラグ（スイッチ1）と `_ekf_reset_triggered` フラグ（スイッチ2）は独立したメンバ変数。両方の更新が同一フレーム内で発生しても、`update()` 内の処理順序は以下の通り: | ✅ 競合なし |
|   |   | **(a)** `userhook_auxSwitch1()` → `set_control_enabled()` → `_control_enabled` を更新 | |
|   |   | **(b)** `userhook_auxSwitch2()` → `reset_ekf_to_initial_state()` → `_ekf_reset_triggered` を設定 & EKFリセット | |
|   |   | **(c)** `update()` → `_control_enabled` を確認して補正出力を決定 | |
|   |   | `reset_ekf_to_initial_state()` が呼ばれた場合、EKF内部状態がリセットされるが、`_control_enabled` には影響しない。逆も同様。 | |
| 5 | **フライトモードとの競合** | CH7/CH8 はフライトモード切替 (CH5) とは別チャンネル。制御ON/OFF や EKFリセットがフライトモード遷移をトリガーすることはない。 | ✅ 干渉なし |
| 6 | **Arm/Disarm との競合** | Arm/Disarm は CH9 (ARMDISARM)。CH7/CH8 操作がアーム状態に影響することはない。仮にEKFリセット中（CH8操作）にディスアームされるケースでも、EKFリセットは純粋に内部状態の書き換えのみであり、モーター出力やアーム状態に影響しない。 | ✅ 干渉なし |
| 7 | **同時操作の安全性** | CH7(LOW→MIDDLE→HIGH) と CH8(モーメンタリ押下) を同時に操作することは物理的に可能（両手操作）だが、各機能は独立して正しく動作する。EKFリセット（CH8）が実行された直後に制御ON（CH7=HIGH）が有効になるケースでは、リセット直後の未収束状態に対して制御が適用される可能性があるが、これは実験Aの目的（収束過程の観測）にとって**望ましい動作**である。 | ✅ 問題なし |

#### 2.2.2 電気的干渉解析（CRSF/ELRS プロトコル）

| # | 項目 | 詳細 | 判定 |
|---|------|------|------|
| 1 | **CRSFチャンネル数** | ELRS 3.x は最大16チャンネル（CRSFプロトコルで1パケットに全CH値を格納）。現在 11CH 使用 + CH7/CH8 追加で計 13CH となり、16CH 制限内。 | ✅ 制限内 |
| 2 | **パケットレート** | ELRS 250Hzモードの場合、全16CHが1パケット（4ms）で更新される。CH7/CH8追加による遅延増加なし。 | ✅ 遅延なし |
| 3 | **PWMクロストーク** | デジタルCRSFプロトコルのため、隣接チャンネル間のアナログクロストークは原理的に存在しない。 | ✅ クロストークなし |
| 4 | **フェイルセーフ時の挙動** | ELRS受信機のフェイルセーフ時、全チャンネルが設定されたフェイルセーフ値（デフォルト: No Pulse / 最後の値保持）にフォールバックする。CH7のNo Pulse時、`RC_Channel::read()` は `trim` 値（約1500µs = MIDDLE）を返す → 制御OFF（安全側）。CH8のNo Pulse時も同様に MIDDLE → エッジ検出トリガー条件を満たさない → 意図しないリセットは発生しない。 | ✅ 安全側に倒れる |

#### 2.2.3 RC_OPTIONS ビットマスク互換性チェック

現在の設定: `RC_OPTIONS = 10336` (10進数) = `0x2860` (16進数)

ビット分解:
| ビット位置 | 値 | 定義 | 関連 |
|-----------|-----|------|------|
| 5 (0x20) | 32 | CRSF/ELRS 関連オプション | RCプロトコル制御 |
| 6 (0x40) | 64 | CRSF/ELRS 関連オプション | RCプロトコル制御 |
| 11 (0x800) | 2048 | CRSF/ELRS 関連オプション | RCプロトコル制御 |
| 13 (0x2000) | 8192 | CRSF/ELRS 関連オプション | RCプロトコル制御 |
| **合計** | **10336** | | |

`RC_OPTIONS` のビットマスクはRCプロトコル層（CRSF/ELRSのパケット解釈、チャンネル順序など）の設定であり、個別チャンネルの AUX_FUNC 割り当て（`RC7_OPTION`, `RC8_OPTION`）とは**完全に独立したパラメータ**である。したがって:

- CH7/CH8 を AUX_FUNC として使用しても `RC_OPTIONS=10336` との競合はない
- `RC_OPTIONS` の値変更は不要

---

### 2.3 要件1: RCスイッチによる制御（補正ゲイン）のON/OFF機能

*   **理由**: 実験C（A/Bテスト）を1回のフライトで実施するため。
*   **対応スイッチ**: **CH7 = 3ポジションスイッチ → `USER_FUNC1` (47)**
*   **仕様**:

    | スイッチ位置 | AuxSwitchPos | PWM範囲 | 動作 |
    |-------------|-------------|---------|------|
    | LOW | `LOW` | < 1200 µs | **制御OFF**: 補正ゲインを強制0、EKF推定は継続 |
    | MIDDLE | `MIDDLE` | 1200 ≦ PWM ≦ 1800 µs | **制御OFF**: 安全デフォルト（意図しないONを防止） |
    | HIGH | `HIGH` | > 1800 µs | **制御ON**: パラメータ `OBS_CORR_GAIN` の値に従って補正を出力 |

    **設計意図**: 3ポジションスイッチのうち HIGH のみを ON にすることで、スイッチを誤って中間位置にした場合に「意図せずON」になるリスクを回避する。LOW と MIDDLE の両方を OFF とすることで、フェイルセーフ時（受信機が No Pulse → MIDDLE にフォールバック）にも安全側に倒れる。

*   **実装詳細**:
    - `UserCode.cpp` の `Copter::userhook_auxSwitch1(ch_flag)` に実装:
      ```cpp
      void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
      {
          // CH7: Observer 制御ON/OFF (3ポジションスイッチ)
          // HIGH=ON, MIDDLE/LOW=OFF
          const bool enable = (ch_flag == RC_Channel::AuxSwitchPos::HIGH);
          observer.set_control_enabled(enable);
      }
      ```
    - `AP_Observer` クラスに `set_control_enabled(bool)` メソッドを追加
    - `update()` 内で `_control_enabled == false` のとき、`current_correction_quat` を単位クォータニオンに、`current_correction_euler` をゼロベクトルにする
    - ON/OFF 状態は OBSV ログに `CtrlEna` フィールドとして記録（要件3参照）
    - **重要な制約**: 制御OFF時も EKF の predict/update サイクルは**継続**する。これにより、制御OFF中の荷物揺れデータも EKF が追跡し、制御ONに切り替えた瞬間に即座に補正が有効になる（コールドスタート不要）。

---

### 2.4 要件2: 飛行中のEKF再初期化（リセット）機能

*   **理由**: 実験A（周波数収束）を1フライト中に何度も試行するため。一度真値に収束したEKFを、再び「誤った初期周波数」と「広い誤差共分散 $P$」に戻す必要がある。
*   **対応スイッチ**: **CH8 = モーメンタリスイッチ → `USER_FUNC2` (48)**
*   **仕様**:
    - **モーメンタリスイッチ**（押した瞬間だけ HIGH、離すと LOW に戻る）の **HIGH→LOW 立ち下がりエッジ** をトリガーとする。
    - **エッジ検出方式**: `static` 変数で前回呼び出し時の `AuxSwitchPos` を保持し、`(prev == HIGH && current == LOW)` で検出。
      - HIGH→LOW の立ち下がりエッジのみで検出：スイッチを「押して離す」の1アクションで1回だけリセットが発動。
      - LOW→HIGH の立ち上がりでは発動しない：スイッチを押しっぱなしにしても連続リセットは発生しない。
      - HIGH→HIGH の連続でも発動しない：スイッチを押しっぱなし→次のフレームでも発動しない。
    - **リセット内容**:
        1. 内部の `ekf_state[axis][3]`（周波数成分 $\omega$）をパラメータの初期値 `_ekf_omega_init` にリセット
        2. 共分散行列 `ekf_P`（4x4）を初期状態にリセット: 対角成分 = `EKF_INIT_COVARIANCE` (10.0)、非対角成分 = 0
        3. 全軸 (X, Y, Z) の EKF 状態を同時にリセット（Z軸はEKF更新をスキップしているが、状態管理の一貫性のため）
        4. `ekf_initialized` フラグは維持する（再初期化に `ekf_init()` を使うのではなく、`reset_ekf_to_initial_state()` メソッドを新設）
    - **リセット通知**: リセットが発生したフレームで OBSV ログの `EKFRes` フィールドに 1 を記録（次のフレームでは 0 に戻るワンショットフラグ）

*   **実装詳細**:
    - `UserCode.cpp` の `Copter::userhook_auxSwitch2(ch_flag)` に実装:
      ```cpp
      void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
      {
          // CH8: EKFリセット (モーメンタリスイッチ: HIGH→LOWエッジ検出)
          static RC_Channel::AuxSwitchPos prev = RC_Channel::AuxSwitchPos::LOW;
          if (prev == RC_Channel::AuxSwitchPos::HIGH && ch_flag == RC_Channel::AuxSwitchPos::LOW) {
              observer.reset_ekf_to_initial_state();
          }
          prev = ch_flag;
      }
      ```
    - `AP_Observer.h` に `reset_ekf_to_initial_state()` を追加
    - `AP_Observer.cpp` にて、`_ekf_reset_triggered` フラグを true に設定し、次の `Write_Observer_Log()` で EKFRes=1 を出力後 false に戻す

*   **備考**:
    - 実験中、操縦者は CH8 モーメンタリスイッチを「カチッ」と1回押すたびにEKFがリセットされる
    - リセットは即座に実行され、次の制御サイクルから新しい初期状態で推定が開始される
    - モーメンタリスイッチの代わりに2ポジションスイッチがCH8に割り当てられている場合、LOW→HIGH（ONにする）→HIGH→LOW（OFFにする）の操作で1回のリセットが発生する（実用上問題なし）

---

### 2.5 要件3: 解析用変数の高解像度ロギング（拡張）

*   **理由**: 論文のグラフ（特に実験A, B, Cすべて）を描画するため、内部変数が確実にログに吐き出されている必要がある。
*   **仕様**:
    *   `Write_Observer_Log()` 関数 (OBSV メッセージ) に以下のフィールドを **追加** する:

        | フィールド名 | 説明 | 単位 | 型 | 追加種別 |
        |-------------|------|------|-----|----------|
        | `CtrlEna` | RCスイッチによる制御ON/OFF状態 (0=OFF, 1=ON) | - | uint8_t | ★新規追加 |
        | `EKFRes` | EKFリセット発生フラグ (1=リセット発生, それ以外は0) | - | uint8_t | ★新規追加 |

    *   既存の OBSV メッセージに以下のフィールドが含まれていることを確認する（不足があれば追加）:

        | フィールド名 | 説明 | 単位 | 型 | 状態 |
        |-------------|------|------|-----|------|
        | `FreqX`, `FreqY`, `FreqZ` | 各軸の推定角周波数 $\omega$ | rad/s | float | (要確認) |
        | `InnX`, `InnY`, `InnZ` | イノベーション（観測残差） $r$ | N | float | (要確認) |
        | `FadeX`, `FadeY`, `FadeZ` | 出力フェードゲイン | - | float | (要確認) |

    *   ロギングレート: 外乱のダイナミクスを十分に捉えられる速度（最低50Hz、理想的には100Hz）を確保する。`AP_Observer::update()` が 400Hz スケジューラから呼ばれる場合、OBSV ログは毎回書き込んで問題ない。

---

## 3. 実装チェックリスト

> **ステータス**: ✅ 実装完了 (2026-05-25)  
> コミット: `a10c7f5906` (AP_Observer), `2cc8c8b372` (UserCode)

### 3.1 AP_Observer側の修正（`libraries/AP_Observer/`） ✅

- [x] `AP_Observer.h`:
  - [x] `void set_control_enabled(bool enabled)` メソッド宣言を追加
  - [x] `void reset_ekf_to_initial_state()` メソッド宣言を追加
  - [x] `bool _control_enabled` メンバ変数を追加（デフォルト `true`: Observerはデフォルトで有効）
  - [x] `bool _ekf_reset_triggered` メンバ変数を追加（ログ用ワンショットフラグ、デフォルト `false`）

- [x] `AP_Observer.cpp`:
  - [x] `init()` で `_control_enabled = true` に初期化
  - [x] `set_control_enabled(bool enabled)` の実装: `_control_enabled = enabled`
  - [x] `reset_ekf_to_initial_state()` の実装:
    - 全軸 (X, Y, Z) で `ekf_state[axis][3] = _ekf_omega_init`
    - `ekf_P` の全要素を0クリア後、対角成分を `EKF_INIT_COVARIANCE` (10.0) に設定
    - 各軸の診断変数と fade 変数もリセット
    - `_ekf_reset_triggered = true` をセット
    - GCS メッセージ送信
  - [x] `update()` 内の補正出力部: `_control_enabled == false` 時に単位クォータニオン/ゼロベクトル出力
  - [x] `Write_Observer_Log()` に `CE` (CtrlEna), `ER` (EKFRes) フィールド追加、ワンショットクリア

### 3.2 ArduCopter側の修正（`ArduCopter/`） ✅

- [x] `UserCode.cpp`:
  - [x] `userhook_auxSwitch1()`: CH7 3ポジションスイッチ → HIGHのみON
  - [x] `userhook_auxSwitch2()`: CH8 モーメンタリスイッチ → HIGH→LOWエッジでリセット

### 3.3 ビルド・テスト ✅

- [x] SITLビルド (`./waf configure --board sitl && ./waf build --target bin/arducopter`)
- [x] Liteオートテスト (`test.CopterObserver`) → 3/3 PASSED (6.14s)
- [x] Mediumオートテスト (`test.CopterMedium`) → 7/7 PASSED (19.85s)
- [x] Pixhawk6C クリーンビルド → 成功 (arducopter.bin 1.6MB)
- [ ] 実機テスト項目（未実施）:
  - [ ] CH7=LOW/MIDDLE → `CtrlEna=0` が OBSV ログに記録されること
  - [ ] CH7=HIGH → `CtrlEna=1` が OBSV ログに記録され、補正が出力されること
  - [ ] CH8 モーメンタリ押下 → `EKFRes=1` が OBSV ログに記録され、推定周波数が初期値にリセットされること
  - [ ] CH7=OFF の状態でCH8リセット → EKFリセットは実行されるが補正は出力されないこと（分離動作確認）

---

## 4. パラメータ設定ファイルへの追加

### 4.1 params_to_set への追加エントリ

現在の `params_to_set` には CH7, CH8 の設定が含まれていない。以下のエントリを追加する:

```python
# === Observer RCスイッチ設定 ===
'RC7_OPTION': (47, 'AP_Int16', 'Observer制御ON/OFF (USER_FUNC1, 3ポジションスイッチ)'),
'RC8_OPTION': (48, 'AP_Int16', 'EKFリセット (USER_FUNC2, モーメンタリスイッチ)'),
```

### 4.2 既存パラメータの変更有無

| パラメータ | 現在値 | 変更 | 理由 |
|-----------|--------|------|------|
| `RC_OPTIONS` | 10336 | **変更不要** | RCプロトコル層（CRSF/ELRS）の設定であり、AUX_FUNC割り当てとは独立 |
| `RC6_OPTION` | 0 (DO_NOTHING) | **変更不要** | あえて使用しない。将来の拡張用に空けておく |
| `RC12_OPTION` 〜 `RC16_OPTION` | 0 (DO_NOTHING) | **変更不要** | 予備チャンネルとして空けておく |

---

## 5. 運用シナリオ（実験ごとのスイッチ操作手順）

### 5.1 実験A: 周波数収束の観測

| ステップ | 操作 | CH7 | CH8 |
|---------|------|-----|-----|
| 1. 離陸前 | 初期周波数をわざとずらした値にパラメータ設定 | MIDDLE (OFF) | - |
| 2. アーム・離陸 | 通常操作で離陸 | MIDDLE (OFF) | - |
| 3. ホバリング開始 | Loiterモードでホバリング | MIDDLE (OFF) | - |
| 4. **EKFリセット** | モーメンタリスイッチを押して離す | MIDDLE (OFF) | HIGH→LOW |
| 5. 収束観測 | 推定周波数が真値に収束するまで待つ（ログ記録） | MIDDLE (OFF) | - |
| 6. **再度EKFリセット** | 必要に応じて繰り返す | MIDDLE (OFF) | HIGH→LOW |
| 7. 着陸 | 通常操作で着陸 | MIDDLE (OFF) | - |

### 5.2 実験B: 突発外乱ロバスト性

| ステップ | 操作 | CH7 | CH8 |
|---------|------|-----|-----|
| 1. 離陸・ホバリング | 制御ONで飛行 | **HIGH (ON)** | - |
| 2. 外乱印加 | 扇風機／棒で荷物に外乱 | HIGH (ON) | - |
| 3. 回復観測 | イノベーションクリッピング後の収束をログ記録 | HIGH (ON) | - |
| 4. 繰り返し | ステップ2-3を繰り返す | HIGH (ON) | - |

### 5.3 実験C: A/Bテスト（制御ON/OFF比較）

| ステップ | 操作 | CH7 | CH8 |
|---------|------|-----|-----|
| 1. 離陸・ホバリング | **制御OFF**で飛行 | **MIDDLE (OFF)** | - |
| 2. 揺れ励起 | 荷物を手で大きく揺らす | MIDDLE (OFF) | - |
| 3. 制御OFF観測 | 揺れが持続する様子を5〜10秒記録 | MIDDLE (OFF) | - |
| 4. **制御ON切替** | スイッチをHIGHに倒す | **MIDDLE→HIGH (ON)** | - |
| 5. 減衰観測 | 揺れが急速に減衰する様子を記録 | HIGH (ON) | - |
| 6. 着陸 | 通常操作で着陸 | HIGH→MIDDLE | - |

---

## 6. 設計上の注意点・制約・TODO

### 6.1 既知の制約

1. **`USER_FUNC` のAUX_FUNCプレフィックス**: `USER_FUNC1/2` は汎用的な名前であり、`RC_Channel.h` のenumコメントにその機能が明示されない。コード内のコメントで機能を明示すること。
2. **CH8 モーメンタリスイッチの物理的制約**: プロポによってはCH8が2ポジションスイッチの場合がある。その場合、ON→OFFの操作で1回のリセットが発生する（実用上大きな問題はないが、2ポジション操作になることを認識しておく）。
3. **Z軸EKFリセット**: Z軸は通常EKF更新をスキップしているが、`reset_ekf_to_initial_state()` では一貫性のため全軸リセットに含める。

### 6.2 将来の拡張案

1. 新規 AUX_FUNC `OBS_ENABLE` (185), `OBS_RESET` (186) を正式に追加し、`USER_FUNC` から移行する（ArduPilot upstream へのコントリビュートを視野に入れた場合）。
2. CH8 のダブルクリック検出（短時間に2回のHIGH→LOWエッジ）で別のリセットモード（例: 周波数のみリセット、共分散は維持）を提供する。
3. CH6 を将来の拡張用（例: ゲイン調整ノブ）として予約しておく。

---

## 付録A: プロポ配線参考図

```
チャンネルマッピング (CRSF/ELRS)
┌─────────────────────────────────────────────────┐
│ CH1  : ロール          (右スティック左右)        │
│ CH2  : ピッチ          (右スティック前後)        │
│ CH3  : スロットル      (左スティック前後)        │
│ CH4  : ヨー            (左スティック左右)        │
│ CH5  : フライトモード  (3ポジションスイッチ①)    │
│ CH6  : (予備・未使用)                           │
│ CH7  : Observer制御    (3ポジションスイッチ②) ★  │
│ CH8  : EKFリセット     (モーメンタリスイッチ) ★   │
│ CH9  : Arm/Disarm      (2ポジションスイッチ)     │
│ CH10 : Loiterモード    (2ポジションスイッチ)     │
│ CH11 : Guidedモード    (2ポジションスイッチ)     │
│ CH12-16: (予備・未使用)                         │
└─────────────────────────────────────────────────┘

---

## 付録B: MCP ツール実装詳細

### B.1 ardupilot-dev-tools MCP サーバー

全 MCP ツール (`build_sitl`, `run_autotest_lite`, `run_autotest_medium`, `run_autotest_full`, `build_pixhawk6c`, `run_ci_pipeline`) は Docker コンテナ内でコマンドを実行し、結果を要約して返す。

**リポジトリ**: `https://github.com/KeitaTK/mcp-servers` (`ardupilot-dev-tools/main.py`)

**修正履歴**:
| コミット | 内容 |
|----------|------|
| `3fb4149f` | 初版 |
| `c3793f1c` | autotest 偽陽性修正、PASSED 検出修正、ログクリーンアップ追加 |
| `5218763b` | コマンド実行ログを `/home/taki/buildlogs/mcp/` に保存 |

### B.2 実行ログ

全 MCP ツール実行時の stdout/stderr が以下に保存される:

```
/home/taki/buildlogs/mcp/
├── 20260525_005334_run_autotest_lite.log
├── 20260525_005410_build_sitl.log
└── ...
```

- ファイル名形式: `YYYYMMDD_HHMMSS_<tool_name>.log`
- 先頭に日時が来るため `ls` で実行順に自動ソートされる
- 各ファイルの先頭に実行コマンドとタイムスタンプをヘッダとして記録

### B.3 Docker 環境

`/home/taki/Ardupilot-Docker/docker/docker-compose.yml` の `ardupilot-dev` サービスに
`/home/taki/buildlogs` のバインドマウントを追加:

```yaml
volumes:
  - /home/taki/buildlogs:/home/taki/buildlogs
```

コンテナ内パッケージ:
- `pymavlink` 2.4.42 (ローカル submodule からインストール)
- `empy` 3.3.4
- `pexpect`, `ptyprocess`, `mavproxy`
- `python` → `python3` symlink