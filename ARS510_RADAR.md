# ARS510 雷達整合（RAV4 2022-25 / C-HR 2021-22 / Yaris）

本分支 = sunnypilot `staging`（b50b82f，sunnypilot v2026.003.000 staging，2026-10-18）＋ [ds-sebastian/ars510-radar](https://github.com/ds-sebastian/ars510-radar)
安裝器（commit 7ff9485，2026-10-18）套用後的結果，profile 改為 `fused`（物件清單＋ACC 目標以 Kalman 融合，可同時追多台車；原預設 `acc` 只輸出雷達 ACC 鎖定的單一目標），對應上游提案 [commaai/opendbc#3860](https://github.com/commaai/opendbc/pull/3860)。

- 不需要 CAN filter 即可讀取雷達；但開啟 openpilot 縱向控制（Alpha）時雷達會被靜音，原廠 PCS/AEB 失效，儀表會顯示 PCS 故障。
- 變更只在 `opendbc_repo/opendbc/car/toyota/`（新增 ars510 套件與 interface.py 結尾的 hook 區塊）與 `opendbc_repo/opendbc/dbc/ars510_acc_target.dbc`，以及下方畫面標示的 chevron_metrics.py，其餘與 staging 相同。
- 裝置安裝網址：`https://installer.comma.ai/dimpleliang/ars510`
- 要還原成純視覺：改裝回 sunnypilot 的 `staging`。
- 更新做法：以上游 staging 重新套用本分支變更後 force push 本分支。舊的 tn-prebuilt 版本保留在 `ars510-tn` 分支。

## 畫面標示（本分支額外修改）

`openpilot/selfdrive/ui/sunnypilot/onroad/chevron_metrics.py`：前車由雷達提供（radarState.leadOne.radar 為 true）時，
前車標記下方一定顯示距離與車速兩行，距離加上「R」前綴，例如 `R 42 m` / `85 km/h`；純視覺前車維持原樣。
需在 設定 → Visuals → Display Metrics Below Chevron 選 Off 以外的任一項才會顯示。

## 畫面標示：雷達看到的每台車（含左右車道）

`opendbc_repo/opendbc/car/toyota/ars510_radar_interface.py` 把雷達每筆追穩的物體（path gate 之前）發到
`customReservedRawData0`（openpilot 保留給 fork 的通道），由 `openpilot/selfdrive/ui/sunnypilot/onroad/ars510_tracks.py`
畫成菱形標記，標示 `R 距離` 與車速：綠色＝離模型預測的行車路徑 1.8 m 以內（我的車道）；紅色＝路徑外（旁邊車道）。顏色只代表車道位置，
不影響 radard 選前車。剛出現、追不到 1.2 秒的目標畫成半透明、只標距離（距離與速度尚未收斂）。
雷達分類為行人（class 4）或機車／自行車（class 6）的目標，距離前加上「行人」「機車」（非繁中介面為 PED / MOTO）；
實車抽樣驗證：行人 7/9、機車 5/6 正確。已經有雷達前車箭頭的那台不重複標示。只影響畫面，radarTracks 與控制邏輯不變。

## 發佈門檻（控制）

`fused` 的 `min_publish_age` 由上游的 60 週期（約 3.6 秒）改為 20 週期（約 1.2 秒），雷達第一次看到就在路徑上的車可更早用於控制。
仍保留 Kalman 速度收斂檢查（publish_speed_std_mps 0.75 m/s）；一般車速下 radard 另以鏡頭前車核對距離（25 % / 5 m）與速度。
剩餘風險在低速（約 4 m/s 以下）：radard 不經鏡頭確認、直接採用正前方 25 m 內最近的雷達點。

## 對向來車過濾（參考 dragonpilot）

參考 dragonpilot 外接雷達介面丟棄對向物體的做法：雷達標為對向（oncoming_flag 或 motion code 2）且對地速度低於 -2 m/s
的目標（ACC 目標、以及離行車路徑 1.5 m 內的對向物體除外，例如窄巷迎面機車）不送給 radard，畫面仍顯示。
實車驗證（0000000e，31 分鐘）：97 個對向旗標目標的對地速度全為負，抽樣影格皆為對向或穿越車輛。dragonpilot 其餘做法（不等收斂、1.5 車道內全送、遠距不限、近距靜止物不擋）
與 ARS510 實測問題衝突（見上游 docs/12），不採用。

## 低速轉角速度放寬（LTA，需 panda 韌體）

Toyota LTA 角度控制的轉角速度上限在 90 km/h 以下放寬：18 km/h 以下 15→25°/s（回正 18→30°/s）、36 km/h 13→17.5°/s、
54 km/h 11→14°/s，90 km/h 以上不變；±95° 指令上限與 EPS 扭力上限不變。由轉角速度表換算的最壞情況橫向 jerk
（平路、定速、軸距 2.69 m、轉向比 13.7，進彎／回正）：18 km/h 0.30／0.36、36 km/h 0.83／1.07、54 km/h 1.51／2.06、
72 km/h 2.05／3.06、90 km/h 2.22／3.85 m/s³（90 km/h 起為原廠表）；用本車 steerRatio 14.3 約低 4 %。
控制端另有 `drive_helpers.clip_curvature` 的曲率 jerk 上限 5.0 m/s³；opendbc `car/lateral.py` 的 MAX_LATERAL_JERK（約 3.6）
Toyota 的 LTA 路徑沒有使用（那是 AngleSteeringLimitsVM 品牌用的）。2026-10-21 依 Codex 審查修正，原文誤寫為「36 km/h 以下 < 1、54 km/h 約 1.5」。
opendbc `car/toyota/values.py` 與 panda 安全 `safety/modes/toyota.h`
同步修改，`safety/tests/test_toyota.py` 通過；`panda/board/obj/panda_h7.bin.signed` 由 sunnypilot/panda 8a16e08 以 debug 金鑰重建
（未修改版本可位元級重現原檔，修改版僅轉角表 16 bytes 不同）。還原：切回前一版，pandad 會自動刷回原韌體。
- 10/27 再放寬 18 km/h 以下：進彎 25→50°/s、回正 30→60°/s（5 m/s 那一列 0.5/0.6 → 1.0/1.2 °/frame），36 km/h 以上不變。
  route 52 18:15:47 路口轉彎：模型 2.3 s 內要求 13→83°，命令被 24°/s 卡住，方向盤最多落後 25°，駕駛接手。
  最壞橫向 jerk 18 km/h 0.59／0.71 m/s³（原 0.30／0.36）；60°/s 低於 LTA 暫停門檻 100°/s。panda 韌體重建，
  與前一版只差這兩個浮點數各 1 byte（加簽章）。注意事項：EPS 扭力在 18:15:48 已到 1400～1700，轉更快可能接近 2500 的收力點。
- 10/29 放寬 36 km/h 那一列：進彎 17.5→25°/s、回正 22.5→30°/s（10 m/s 列 0.35/0.45 → 0.5/0.6 °/frame），25 m/s 起不變。
  10/27～28 共 10 趟（openpilot 自己轉向約 64 分鐘）：EPS 扭力最高 2242（2500 的 90 %）、側向加速度最高 2.56、jerk 最高 1.62；
  兩次 steerSaturated（10/27 21:34 40 km/h 48°、10/28 13:13 46 km/h 40°，側向 2.2 m/s²、EPS 1900～2100）都是命令卡在轉角速度
  上限、落後 2.4～6°，扭力和側向加速度都不是瓶頸。內插使 54 km/h 變 19/24°/s、72 km/h 變 13/19°/s。最壞橫向 jerk：
  36 km/h 1.18／1.42、54 km/h 2.04／2.59、72 km/h 2.53／3.54 m/s³，都低於原廠 25 m/s 列的 3.85。分析腳本 tools/ars510/lat_recent.py。

## 低速 LTA 扭力上限放寬（需 panda 韌體）

LTA 開始收力（TORQUE_WIND_DOWN）的 EPS 量測扭力門檻：70 km/h 以下 1500→2500（2026-10-21 先放寬到 2000，10/23 再到 2500），70–80 km/h 漸變，80 km/h 以上維持 1500。
駕駛手力門檻 150 不變；橫向加速度仍受控制端 `drive_helpers.clip_curvature` 的 3.0 m/s²（加路面傾斜補償 g·roll，
約 3 % 傾斜時單側約 3.6）上限限制，Toyota LTA 沒有另一道 3.6 的直接橫向加速度保護。2026-10-21 依 Codex 審查修正：原文「3.6 m/s² 上限」
與「接手難易度不變」皆過度推論，多出的 500 單位以 EPS 約 1500 units/s 卸力約需多 0.33 秒，接手感受可能改變；
2000 平台一路到 70 km/h 目前沒有 EPS 能力或實測橫向加速度證據（詳見 tasks/lta-curve-torque-codex-2026-10-21.md）。
2026-10-23 實測：13:27 下匝道（58→48 km/h，需 2.7 m/s²）EPS 扭力到 1964 後觸發收力，扭力掉到 ~850、方向盤停住約 1 秒，再爬回 1971 又收力，形成約 1 Hz 的鬆緊循環，角度落後 9–11° 直到駕駛接手；12:54 匝道（2.37 m/s²）峰值 1934 未觸發。2000 約只撐到 2.4 m/s²，故再放寬到 2500。LTA 模式下 EPS 自身的扭力上限仍無數據，需上車驗證。控制端 `car/toyota/carcontroller.py`、
`values.py` 與 panda `safety/modes/toyota.h` 同步修改，Toyota 安全測試通過，panda 允許值於各車速皆 ≥ 控制端收力點。

## 駕駛接手時 LTA 轉角命令跟隨實際角度、放手後以漸增的角度差帶寬接回

2026-10-24 18:04 右彎匝道（30 km/h）：駕駛握著方向盤轉到 −97°，模型路徑同時放寬到 −46°，命令已拉到 −68°，
放手瞬間方向盤以 138°/s 往左回到 −45°（EPS 扭力 2288→394：駕駛扭力歸零、輔助消失，輪胎回正力矩把方向盤甩回），
駕駛感受為「突然往左的大力道」。
修正（`carcontroller.py`，10/24 第二版）：
- 接手狀態有遲滯：駕駛扭力 >`OVERRIDE_ENTER_TORQUE`（150，與 EPS LTA 收力的駕駛扭力門檻相同）持續 0.1 s 才進入，
  <`OVERRIDE_EXIT_TORQUE`（100）持續 0.2 s 才離開。10/27 修正：第二版以 steeringPressed（>100）進入、<50 離開，但駕駛手搭方向盤時
  扭力中位數 50～65、p75 約 100，10/25～26 資料顯示接手狀態佔啟用時間約 45%（其中 18～27% 只是遲滯撐住），期間命令＝量測角、
  openpilot 不轉，彎道反應明顯變慢。新門檻下以同批資料回算接手佔比降為 11～19%。
- 接手期間轉角命令直接等於量測角（仍受速率限制，駕駛轉得比速率表快時命令會落後幾度）。
- 放手後 `OVERRIDE_RELEASE_RAMP_FRAMES`（1 s）內，命令被限制在「量測角 ± 帶寬」內，帶寬由 0 線性放大到
  `OVERRIDE_RELEASE_BAND_DEG`（15°）；EPS 位置控制的出力大致與命令−實際的差成正比，直接限制這個差就是限制力道
  （第一版限制的是命令速率，駕駛仍輕扶時差距照樣累積，改掉）。之後再過原本的轉角速率限制，panda 檢查不受影響。

## 軟停（停車最後一段放鬆煞車）

2026-10-25 IMU 量測 8 次 openpilot 自行停車：停住瞬間減速度 −0.22～−0.66 m/s²，停住後車身回彈 +0.27～+0.36 m/s²、jerk 8～13 m/s³（「最後一頓」）。
原因：v < 0.3 m/s 進入 stopping 後以 1 m/s³ 把請求加重到 stopAccel −2.0，最後半秒煞車反而變重。
修正（`openpilot/selfdrive/controls/lib/longcontrol.py`）：
- ~~pid 狀態下 v < 3 m/s 時減速請求上限隨車速遞減~~（10/27 移除，見下方「停車後往前滑」）。
- stopping 狀態下 0.05 < v < 1.0 m/s 時，請求以 1.5 m/s³ 放鬆到 −0.3；完全停住（或 v ≤ 0.05）後才照原本加到 −2.0 停穩。
以實際 LongControl 搭配 0.3 s 延遲模型模擬：停住瞬間減速度 −0.8～−1.3 → 約 −0.34；多滑 0.15～0.7 m、最多約 1 s；
規劃 −2.0（緊急）時不受影響（停住瞬間 −1.95），−1.5 時部分套用（停住瞬間 −0.73）。
- 10/27 補：輪速在約 0.2～0.3 m/s 時就掉到 0（0.22 → 0.03 m/s 只花 0.2 s），openpilot 以為已停住、開始把請求加重到 −2.0，
  但 IMU 顯示車身還在減速約 0.7 s，真正停住時減速度已到 −0.4、緊接 +0.2 的頓挫（陀螺儀俯仰角速度同步出現尖峰），這才是
  「最後那一下」。改為輪速掉到靜止後，先維持 −0.3 一段時間（最後滾動速度 / 0.3 + 0.2 s，限 0.3～1.2 s）再加重。
  模擬（輪速低於 0.25 m/s 讀 0、0.3 s 延遲）：真正停住時的減速度 −0.63 → −0.30。
- 10/27 停車後往前滑（安全問題，已修）：route 4f 15:04 前車已停，車速在 0.34～0.46 m/s 掛了 3 秒、距離 2.9 → 1.6 m，駕駛自己踩煞車。
  原因兩個疊加：(1) pid 狀態的減速上限在 0.9 m/s 把規劃的 −1.04 壓成 −0.63，規劃器內部速度是用自己的 a_target 積分的，
  以為車已經慢下來，後面只要 −0.13～−0.28；(2) 爬行速度下 Toyota PCM 對小的減速請求幾乎不煞（怠速蠕行：
  命令 −0.2～−0.35 只得到 0～−0.1 m/s²），而規劃器要 v < 0.3 m/s 才進 stopping，車就停在 0.3～0.6 m/s 一直往前滑。
  同一天 15:01、15:03 也有同樣的掛速，只是最後有停下。
  修正：移除 pid 狀態的減速上限（approach 回到原廠）；stopping 的放鬆改為閉迴路——還在滾動時只有真的在減速（aEgo < −0.1）
  才往 −0.3 放，沒在減速就以原廠 1 m/s³ 加重直到減速；輪速掉 0 後的維持每次停車只用一次；
  另加爬行防呆：0.05～0.6 m/s、規劃連續 1 秒要求減速（a_target < −0.05）但 1 秒內車速降不到 0.1 m/s，
  就鎖定為停車請求（進 stopping），直到規劃要加速（a_target ≥ 0.1）、踩油門/煞車或解除才放開。
  以 10/25 原廠版 8 次停車與今天 5 次回放：新版停車時間與原廠相同（舊版多 0.3～1.6 s），15:04 那次新版會停下、舊版不會。

## LTA 故障迴避（方向盤超出範圍或快速回正時暫停 LTA 請求）

2026-10-25 市區行程 46 次 `steerTempUnavailableSilent`（LTA 停 2 秒）：全部發生在駕駛手動把方向盤轉過 ±95°（最多 400°）後放手、
方向盤以 150–200°/s 自行回正的過程，前一版（route 48）也有同類事件。修正（`carcontroller.py`）：方向盤轉速 ≥ `LTA_CUT_RATE`
（100°/s，同 Toyota 扭力控制的 MAX_STEER_RATE）或角度超過 `LTA_CUT_ANGLE`（100°，高於 LTA 指令上限 94.95°，openpilot 自己打滿時不會觸發）
時送 LTA 請求 = 0、命令 = 量測角（與未啟用時相同，panda 允許），條件解除後再保持 0.2 s，然後經放手接回帶寬恢復。
以 10/25 資料回放：48 次故障前全部滿足觸發條件；觸發時段前後 1 秒內一定有駕駛出力，不會在 openpilot 自己轉向時誤觸發。

## 盲點偵測加速（Enhanced BSM，UDS 輪詢）

原廠 `BSM`（0x3F6）只以 1 Hz 廣播（10/24 實測 59 秒 64 幀），穿越盲區不到 1 秒的車可能整個漏掉、顯示延遲最多 1 秒。
移植 sunnypilot/opendbc#502（arne182 / rav4kumar，未合併）：離開 R/P 檔時對左右盲點 ECU 各送一次
DiagnosticSessionControl 0x60（`0x750` 子位址 `0x41`/`0x42`），之後每側每 0.2 秒讀 `0x21 0x69`，回應 `0x758`（DBC `DEBUG`：
`BLINDSPOTSIDE`、`BLINDSPOTD1/D2`，>35 視為有車；1–31 是雜訊帶）。10/25 起停車時不再離開診斷模式（原廠 1 Hz 廣播在 16 km/h 以下大多不報：10/25 資料 <6 km/h 時 eBSM 189 次有車、原廠 169 次沒報，
靜止時後照鏡亮但 comma 不顯示），改為倒車或 P 檔 0.2 s 才送 0x10 0x01 離開。10/25 再取消 >6 m/s 才進入的限制：只要不在 R/P 檔 0.2 s 就進入（N/D 靜止也可），
PR 的限制是因為原廠環景攝影機在低速開啟診斷模式時會停擺，本車環景為副廠，不受影響；進入模式只在轉換時送一次（重送會讓 ECU 間歇丟失偵測狀態）。
結果與原廠 1 Hz 訊號 OR 後寫入 `leftBlindspot`/`rightBlindspot`，顯示邏輯不變。只在 openpilot 縱向控制下啟用
（原廠縱向時 panda 不放行 0x750）。panda `toyota.h` 只放行這 6 則固定內容的 UDS 訊息，安全測試已補。
程式：`opendbc/sunnypilot/car/toyota/enhanced_bsm.py`、`toyotacan.py`、`carstate.py`、`carcontroller.py`、
`dbc/generator/toyota/_sp_debug_toyota.dbc`。

## panda 韌體重建方式（prebuilt 分支，`panda/board/obj/panda_h7.bin.signed` 進版控）

本分支是 prebuilt，車機不會自己編 panda 韌體；`safety/modes/toyota.h` 改了之後**必須**重建並提交
`panda/board/obj/panda_h7.bin.signed`，否則車上的 panda 仍是舊規則（10/23 的 2500 門檻和 10/24 的 eBSM UDS 放行
一開始就漏了這步，10/24 晚上補上）。重建步驟（可位元級重現 10/21 的檔案）：
sunnypilot/panda 8a16e08、去掉 .git（版本字串才會是 `DEV-unknown-DEBUG`）、xPack arm-none-eabi-gcc **13.2.1**（14.x 產生的
二進位不同）、`PYTHONPATH` 指向本分支的 `opendbc_repo`、`scons -j4`，debug 金鑰簽章；只複製 `panda_h7.bin.signed`
（其餘 obj 檔是 sunnypilot 原始 prebuilt 產物，10/21 起就沒跟著更新）。panda 端的 EPS 扭力門檻比較的是乘過 EPS_SCALE
（本車 73 %）的值：panda 2000 ≈ 原始 2740、2500 ≈ 原始 3425，所以 panda 門檻一向比控制端寬，舊韌體不會擋 LTA，
但會擋 eBSM 的 0x750 診斷訊息。

## 儀表 TBT（轉向導航）測試

12.3 吋儀表有「設定 → 計量設定 → 彈出顯示 → TbT 導航」，但台灣 Garmin 主機（原廠導航與 iOS 27 CarPlay）都不會讓儀表跳出箭頭。
`toyota_2017_ref_pt.dbc`（Toyota 參考 DBC）有主機 AVN 系列訊息，其中 `AVN1S13`（0x3A5）為 TBT/HUD 導引：HUD_DISP、DT_GP/DT_UNIT（距離）、
DIR_TURN（11.25° 一格）、ROTARY、ROAD_TP1/2、COMPASS、DIR；`AVN1S17`～`AVN1S19` 為車道導引。架構上主機在 Bus 3、儀表在 Bus 5、
comma 在 Bus 1（前鏡頭），閘道器通常只把主機訊息轉往儀表，從 Bus 1 送出的 0x3A5 很可能到不了儀表，此測試用來確認。
- `_sp_debug_toyota.dbc` 加入 AVN1S13；panda `toyota.h` 縱向 TX 白名單加入 0x3A5（bus 0，不檢查 relay），safety 測試同步。
- `opendbc/sunnypilot/car/toyota/tbt_test.py`：只有 `/data/ars510_tbt_test` 存在、靜止且 P 檔、openpilot 未啟用時，以 5 Hz 送 0x3A5，
  每 40 s 循環四個 10 s 階段（右 90°/單位 1、左 270°/單位 2、迴轉 180°/單位 3、直行 0°/單位 0，距離值 30），目前階段寫入
  `/data/ars510_tbt_test.phase`。
- `tools/ars510/lat_limits.py <route 或 rlog…>`（10/28）：仿 sunnypilot/opendbc#299 的驗證方法，只計 openpilot 自己轉向的時間
  （LTA 啟用、前後 0.5 s 無駕駛出力、非變道、> 3 m/s），按車速列出命令角（carOutput 經車輛模型）與 IMU 實測的最大側向加速度、
  0.5 s 窗口側向 jerk，對照 ISO 3.0 m/s²／5.0 m/s³，並檢查 LTA 每步命令是否超過 ANGLE_LIMITS。route 52 兩段測試：
  最大 2.41 m/s²、0.95 m/s³，命令最快 33°/s，沒有超表。
- `tools/ars510/can_capture.py <標籤> <秒數>`：錄 pandad 的 can，輸出 `/data/ars510_can_<標籤>.csv.gz` 並列出各匯流排上的 AVN 訊息。

## 自動上傳完整 rlog

`openpilot/system/loggerd/uploader.py`：熄火（offroad）且非計量網路（WiFi）時，自動上傳每段的 rlog（含雷達 CAN），
不必在 comma connect 點選；由最新的行程開始上傳。完整解析度影片（fcamera/ecamera/dcamera）仍需從 connect 要求。每分鐘約 11 MB。

## 雷達前車黏住（radard，只放比鏡頭近的）

2026-10-20 四趟行程（行駛 106 分、鏡頭有前車 88 分）分析：鏡頭看到前車時 90% 由雷達接手，0–30 m 93%、30–60 m 90%、60 m 外 56%。
雷達與鏡頭的距離中位差 +0.17 m，橫向偏差 < 0.5 m（p90）。前車在雷達與鏡頭之間切換每分鐘 7.1 次，主因不是雷達掉追：
高速時鏡頭的距離估計常偏遠（雷達 33 m、鏡頭 47 m），超出 radard 的 25 % 比對範圍，前車距離就跳 10 m 以上。
`openpilot/selfdrive/controls/radard.py`：上一刻的雷達前車在比對失敗時保留，條件為距離差 < max(50 %, 8 m)、
**不比鏡頭前車遠**（+2 m 內，鏡頭先看到的較近車輛——切入——仍立即接手）、與鏡頭前車橫向差 < 1.5 m、速度檢查同原本。
重播（實際 radard 程式碼、相同 106 分鐘）：雷達接手 90.0→91.1%，切換 7.1→5.0 次/分，前車距離跳 > 5 m 22.8→18.1 次/分。
不限遠近的版本切換可降到 3.2 次/分，但 178 次保留中有 117 次之後證實前方有較近的車（切入），故不採用。
解碼器參數（發布年齡 10、速度標準差 1.0、路徑門 3.5 m）重播改善 < 1 %，維持不變。

## 遠端連線

- **ssh.comma.ai**（comma prime）：Mac `~/.ssh/config` 設 `Host comma-*` 經 `ProxyCommand ssh %h@ssh.comma.ai -W %h:%p`，
  金鑰為裝置上 GitHub 帳號抓到的那把；`ssh comma-<dongle_id>`。
- **Tailscale**：`scripts/tailscale/install.sh` 把靜態 arm64 版裝進 `/data/tailscale`（AGNOS 更新只換系統分割區，`/data` 保留），
  `launch_chffrplus.sh` 每次開機呼叫 `scripts/tailscale/start.sh` 帶起 `tailscaled`（未安裝則直接跳過，不影響 openpilot 啟動）。
  首次需在裝置上 `sudo /data/tailscale/tailscale --socket=/data/tailscale/tailscaled.sock up --hostname=comma` 並開連結登入。
  兩者都要 comma 開著（車子發動或熄火後尚未自動關機）。

## 車道內機車優先（radard）

鏡頭的前車常是機車前方更遠的汽車（台灣路況）。10/20–10/21 共 206 分鐘紀錄：雷達類別為機車（163|3 = 5/6）、已收斂、
落在模型路徑 1.0 m 內、比目前前車近 5 m 以上的目標 33 次 / 91 秒；10/21 的 6 次全部以影像核對為車道內機車。
`openpilot/selfdrive/controls/radard.py` 訂閱雷達顯示訊息（customReservedRawData0）取得每個目標的類別；符合上述條件連續 0.5 秒
後以該機車為 leadOne（有雷達距離與速度），保留條件放寬為路徑 1.3 m 內且仍比鏡頭前車近；低於 4 m/s 由原本的低速規則處理。
對地速度 < 1 m/s（路邊停放）不採用。重播：37 次 / 82 秒，其中 28 次當下距離小於期望跟車距離（會減速拉開距離），約每 5 分鐘市區行駛 1 次。
觸發時 cloudlog 記錄 `ARS510 two-wheeler lead override`。
2026-10-21（Codex gpt-6-astra 審查後）：override 的機車不再借用鏡頭前車的 modelProb（改為 0；MPC 只用 modelProb 判斷 FCW，不影響煞車權重），
也不再把機車 ID 寫進一般鏡頭／雷達比對的 keep_ids（之前會讓下一輪一般比對先選到同一台機車、再與自己比距離而清掉 active，
兩套保留條件互相混用）；模型沒有前車資料時一併清掉機車確認計數。尚未以 rlog 重播驗收，拒收門檻：真機車額外掉前車或切入接手延遲增加 > 0.10 秒。
同日 Patch 2：radard 把顯示訊息（customReservedRawData0）改為 optional service（ignore_alive / ignore_avg_freq / ignore_valid，
與 plannerd 的做法相同），顯示 publisher 停止或延遲時不再拖垮 radarState.valid；類別只採用 150 ms 內的有效訊息，避免舊類別無限沿用。
顯示排程延遲超過 150 ms 時機車 override 會被撤銷，若實際健康路線常超過此值，先修傳輸而不是放大 TTL。
同日 Patch 3：`match_vision_to_track` 的鏡頭／雷達比對分數由三個 Laplace 機率相乘改為負對數相加（排名在正常數值下完全相同，
20000 組隨機案例差異 0），避免機率乘積下溢成 0 後所有候選同分、由字典順序決定人選。原本會下溢的情況以 `ARS510 match underflow`
記進 cloudlog（累計次數，每 5 秒最多一行），rlog 可回答實際發生頻率；門檻與 sticky 條件不變。

## 原廠 PCS 警報改從雷達匯流排讀（同步上游 ars510-radar fecbee8，對應 opendbc#3860）

開啟 openpilot 縱向控制（Alpha）時原廠雷達被靜音，儀表匯流排的 PCS_HUD 不再回報 FCW；雷達自己的匯流排（bus 1）仍會送
0x100 PCS_STATE。上游安裝器 fecbee8 起在 `ars510_radar_interface.py` 的 hook 多包一層 `CarInterface.update`，直接從原始
CAN 幀讀 0x100 byte 0 bit 1（`ars510/support.py` 的 `pcs_alarm`）併進 `carState.stockFcw`，雷達通電第一幀 FF FF 忽略；
carstate.py 維持原樣。`ars510_acc_target.dbc` 同步為 PR 版本（多 POWER_UP 訊號）。openpilot 對 stockFcw 只記錄不顯示警示。
上游另有選用的 `openpilot/radard_vision_fusion.patch`（radard 融合鏡頭前車速度），本分支未套用。

## 遠距停止車輛（far_slow_keep_mps）

60 m 外的遠距規則原本只放行 ACC 目標的軌跡（遠距移動目標有假接近速度）。對地速度 < 3 m/s 的靜止／慢速目標速度穩定，
且 radard 仍要鏡頭前車同意才採用，故改為在路徑門內繼續發布（`far_slow_keep_mps=3.0`）。10/20–10/22 三天重播：60 m 外雷達接手
59.6→61.0%，幽靈前車與切換次數不變；三次「以 40–50 km/h 接近停止車輛」的事件中一次雷達提前到 77 m（原 69 m），
另兩次原本就由 ACC 目標在 81 m／53 m 接手。注意：這顆雷達在高速時會刪掉大部分新出現的靜止物（上游 docs/08：>10 m/s 僅保留 7 %），
高速公路上的停止車輛主要靠 ACC 目標與鏡頭，目前尚無 >80 km/h 的實測事件。

- 10/27 晚 停住瞬間的鈍感（route 52 高頻 IMU 兩次停車，停住瞬間 −0.35～−0.45 m/s²、回彈 +0.22～+0.23）：
  (1) 18:17:06：輪速在約 0.2 m/s 就讀 0，但車以 −0.06 m/s² 又滑了 1.3 s；計時式維持只撐 0.75 s 就開始加重，
  煞車剛好在真正停住時咬上。改為用 locationd 的校正後縱向加速度（已扣重力）在輪速失效後積分真實車速，
  維持請求直到真的停住（v ≤ 0.02 m/s）才加重到 −2.0；維持期間若濾波後減速度 < 0.03（沒在減速）就照原廠速度加煞車，
  最長 2.5 s；pose 無效時退回原本計時式。回放兩次停車：真停時間判斷誤差 < 0.1 s。
  (2) 18:18:35：Toyota carcontroller 的 PID 積分在 stopping 時凍結，爬行階段累積的 −0.13 讓 −0.30 的請求變 −0.43。
  stopping 時改為以 0.5 m/s³ 把積分歸零，請求照原值送給 PCM。

- 10/27 深夜 停住前加煞車改為 Toyota 舊調校的 0.3 m/s³（route 56 21:40：車以 −0.09 慢慢減速、約 3 s 會停，卻被判斷
  「沒在減速」以 1 m/s³ 加到 −0.77，停住 −0.45、回彈 +0.30）。上游 openpilot 已拿掉各車型 stoppingDecelRate 改固定 1.0；
  comma 以前的 TSS2 值（dragonpilot 仍用）是 0.3「平順地達到停車目標」。真正停住後仍以 1.0 加到 −2.0；
  完全沒在減速（aEgo > −0.03）超過 1 s 時改回 1.0（擋怠速蠕行）。本次停車中曾加過煞車就不再放回 −0.3
  （模擬：下坡 + 蠕行時放回會一直沒停住）。規劃要的煞車比目前請求多 0.3 以上時直接照規劃（stopping 狀態原本忽略規劃）。
  模擬 486 種煞車模型：全部停住並加到 −2.0，最壞多滑距離與前版相當，柔和停車比例提高。

- 10/28 停住前的煞車咬合帶改慢加（route 59，駕駛回報「有改善但希望更好」）。五次停車量測：停住瞬間減速 −0.35～−0.47、
  回彈 +0.2～0.24；唯一柔和的一次（−0.07 / +0.08）請求只到 −0.30。原因：0.2～0.3 m/s 蠕行時 PCM 對 −0.33 以上的請求
  完全不煞，請求以 0.3 m/s³ 在 0.4 s 內穿過 −0.33～−0.45，加上 0.35～0.45 s 延遲，煞車一咬住就是 −0.35 以上。
  改法：(1) 請求到 −0.33 以後改以 0.1 m/s³ 加（延遲造成的過衝 0.12 → 0.04）；(2) 煞車開始咬合改看濾波後的 pose 減速度
  （−0.08，比輪速 aEgo 早約 0.3 s），一偵測到就停止加煞車；(3) 完全沒減速改回 1.0 m/s³ 的門檻 1 s → 1.5 s。
  代價：蠕行到停住最多多約 1 s（0.2 m/s 約 20 cm）；規劃要求多煞 0.3 以上時照樣直接跟隨。模擬弱煞車情境多滑 0.5～1.5 m。
  路測（route 5a，6 次）：咬合帶內沒有中間煞車力——請求 −0.30 停住也可能 −0.4，請求 −0.39 也可能 −0.29。

- 10/28 煞車持續咬住（brake hold）。route 5a 的 6 次停車分兩群：柔和的 3 次（停住 −0.16～−0.22、回彈 ≤ 0.11）都是從
  0.6 m/s 一路減速到停、摩擦煞車沒放開；頓的 3 次（−0.29～−0.43、回彈 0.23～0.55）都先在 0.4～0.5 m/s 停滯：規劃器把
  請求放鬆到 −0.05～−0.3，PCM 在請求高於約 −0.31 時放掉煞車，車子蠕行，之後在蠕行速度重新咬合就是一記抓煞（請求 −0.33
  得到 −0.5）。改法：PID 狀態下 v < 0.8 m/s 且規劃要求 < −0.08 時，請求下限 −0.33 並閂住（直到規劃要求 > +0.1、踩油門
  或 v > 1.0），讓煞車一路咬到停住；SOFT_STOP_ACCEL 也從 −0.30 改 −0.33（避免 stopping 放鬆到放掉煞車）。
  代價：接近停止前車時會比規劃早約 1 m 停住、停住前減速約 −0.3（不是零），但不再有重新咬合的抓煞。
  規劃要求 −0.05 跟著蠕行的前車時不閂（門檻 −0.08）。
  路測（route 5b）：煞車確實全程咬住，但 0.8 m/s 就閂 −0.33 太早（規劃只要 −0.13、離前車 7 m 就開始，比規劃早約 2 m
  停住）、停住仍 −0.37；駕駛稱讚的 13:57 那次停住前減速只有 −0.08（下坡）。咬住時 PCM 大致照請求給力（−0.33 → −0.37、
  route 5a 的 −0.18 → −0.2），因此改 0.6 m/s 以下、規劃 < −0.1 才閂，下限 −0.28，SOFT_STOP_ACCEL 同 −0.28。

## 定速：儀表設多少就跑到多少（openpilot 縱向）

10/27 高速：儀表設 115、前方無車，儀表只到 112。Toyota PCM 回報兩個設定值：儀表顯示的 `UI_SET_SPEED`（115）與內部目標
`SET_SPEED`（108，實際車速），openpilot 原本照後者。PCM 內部值一律比儀表低約 6%（30→27、45→41、80→75、115→108），
但本車儀表只多報約 3.7%（輪速與 GPS 一致 0.999，實際 108.0 km/h 時儀表 112），所以實際停在比儀表設定低約 3 km/h。
修正（`opendbc/car/toyota/carstate.py`，`DASH_SPEED_RATIO = 1.037`）：openpilot 縱向時目標速度 = 儀表設定 ÷ 1.037
（115 → 實際約 110.9 km/h）；comma 畫面車速 `vEgoCluster` 改為 vEgo × 1.037 與儀表一致（原為 1.015）。原廠 ACC 不受影響。

## 轉彎不加速、市區不自動變道（10/27）

10/27 21:41（route 56）：45 km/h 打左燈準備路口左轉，(1) 自動變道（打燈 1 s 後不用推方向盤）把它當變換車道，1 s 後開始往左移；
(2) 轉彎時前車離開視野，openpilot 往設定速度加速（+0.5 m/s²），當時 31～32 km/h，剛好高於「打燈不加速」的 30 km/h 門檻。
- `longitudinal_planner.py`：打燈不加速門檻 30 → 50 km/h；方向盤 > 45°（< 30° 解除）且車速 > 3 m/s 時不加速，與是否打燈無關
  （3 m/s 以下不擋，方向盤打著也能起步）。回放 21:41：21:41:42 的 +0.53 會被壓成 0。
- `desire_helper.py`：計時式自動變道只在 60 km/h 以上；以下要推方向盤才變道。高速公路行為不變。

## 自動上鎖／解鎖（移植 sunnypilot/opendbc#354，10/28）

rav4kumar 的 PR（源自 dragonpilot，感謝 AlexandreSato、cydia2020）：D 檔、車門關閉、車速 ≥ 10 km/h 時上鎖一次；換到 P 檔時解鎖。
透過 0x750 子位址 0x40（主車身 ECU）的 UDS IO 控制（`40 05 30 11 00 80` 上鎖、`… 40` 解鎖）。
- `opendbc/sunnypilot/car/toyota/auto_lock_unlock.py`；只在 openpilot 縱向、非 SecOC 時啟用（panda 只在此時放行 0x750）。
- 先以檔案開啟：`/data/ars510_door_lock`（內容 `lock`、`unlock` 或留空 = 兩者都要），刪掉即關閉。
- 旗標改號：PR 的 8/16 已被上游占用；`SP_ENHANCED_BSM` 32 → 128（上游 #440 用 32、#502 用 64），AUTO_LOCK 256、AUTO_UNLOCK 512。
- panda `toyota.h` 0x750 放行加入這兩則固定訊息，safety 測試同步（含其他值／其他 IO 代號應被擋）；panda 韌體重建。
- 未確認：本車 comma 接在 ADAS 匯流排，0x40 主車身 ECU 在另一條匯流排，要靠閘道轉送診斷位址才會動作；不動作時無副作用。

- 10/28 實車：dragonpilot 的 KWP 指令（0x30 / 0x11）被本車車身 ECU 拒絕（回 7F 30 11 服務不支援）。先確認閘道路徑通
  （0x750／0x40 tester present 6 ms 回覆、讀到 ECU 序號），再用 GTS 資料庫（kaikozlov/toyota-diagnostics）的 Active Test
  實測：擴展診斷模式 `10 03` → `2F 2810 03 01 80`（Door Lock ON）門鎖上；`2F 2811 03 01 80`（Door Unlock ON）解鎖；
  之後 `2F 28xx 00 80` 歸還控制。改為三步序列（session、50 ms 後指令、1 s 後歸還），panda 白名單加這五個訊息並重建韌體。
  另加：靜止（< 1 m/s）時有車門打開（駕駛從裡面拉把手）→ 全車解鎖，方便乘客。
  中控主機 0x7D0 在鏡頭匯流排上沒有回應（亮度那條路要走 OBD 口）；儀表 0x7C0 有回應。

## 雷達匯流排 DBC（除錯用，10/29）

`opendbc/dbc/toyota_tss2_ars510.dbc` 取自 ds-sebastian/opendbc `ars510-acc` 分支（ddcc0b07，作者準備送 commaai/opendbc 的版本），
15 個訊息：0x100 PCS_STATE、0x190 雷達週期、0x191–0x194 目標摘要、0x195 EVENT_OBJECT、0x196 EVENT_STATE（REQUIRED_DECEL
每格 0.095 m/s²、RADAR_FAULT）、0x235–0x23B ACC 目標、0x202 / 0x24F、0x680。只給 Cabana 與離線分析用，控制程式沒有讀它
（雷達介面在 Python 自行解碼）；0x80 物件清單是 ISO-TP，不在 DBC 裡。Cabana 開 bus 1 選這個 DBC 即可。

## 雷達模式改為 acc_summaries ＋ 物件清單只給畫面與機車（10/29）

上游 ars510-radar 10/25（#231）撤下 `fused`：第二台 RAV4（DustyL，issue #66）新的 38 趟裡 `fused` 每小時比 `acc` 多約 2 秒不必要
煞車，含一次對距離不變的物件清單軌跡（接近速度讀 23.7 m/s）要求 −3.31 m/s²，主因是物件清單 0x80 速度的 1–10 秒偏移（多在 40 m 外）。
作者在 issue #66 建議改用 `acc` 或 `acc_summaries`。本分支改為：
- radarTracks（radard 與控制用）＝`acc_summaries`：ACC 目標，加最多 2 台 40 m 內的摘要車；同步上游新版
  （摘要車速度改為以雷達 0x190 週期時鐘計時的等速 Kalman 濾波，取代 8 點最小平方斜率）。
- 物件清單（FUSED_CONFIG，含對向車過濾、發佈年齡 20、遠距停止車）照常解碼，只用於：(1) 畫面標示每個目標（不變），
  (2) 它會發佈的二輪車（類別 5/6、已收斂、過路徑門）且 ≤ 40 m 者，以 trackId `1<<30 + id` 加進 radarTracks；與 ACC／摘要點
  相距 1 m／0.5 m 內視為同一物件不加。radard 依 trackId 範圍直接視為二輪車，機車優先邏輯（路徑 1.0 m、比鏡頭前車近 5 m、0.5 s）不變。
- 遠距停止車（同日補回，加距離交叉驗證）：物件清單發佈的 40 m 外、對地速度 < 3 m/s 的軌跡，**過去 1 秒距離的最小平方
  變化率與 vRel 相差 < 2 m/s** 才以 trackId `1<<29 + id` 加進 radarTracks（需 ≥ 10 筆、跨 ≥ 0.9 s）。理由：上游那次 −3.31 假煞車
  是距離不變、卻讀成接近 23.7 m/s（自車約同速），只看速度會像停止車；距離變化率約 0，與 vRel 差約 24 m/s 被擋下。
  代價：首次看到後約 1 秒才放行（100 km/h 約 28 m）。radard 仍須與鏡頭前車相符才採用。合成測試：假接近 0/25 次放行、
  真停止車 0.96 s 後放行；重播 route 59 找到 2 個路徑內停止目標（48.8 m vRel −13.9、66.3 m −8.3），route 5a 無。
  同日改為遲滯：觀察 1.5 s（≥ 15 筆、跨 ≥ 1.35 s），進入 < 3 m/s、維持 < 4 m/s。19 段紀錄中靜止目標 |距離變化率 − vRel|
  中位 0.5–1.2、p90 2.2–3.8 m/s（20–80 m，距離雜訊約 1 m）；行駛中候選週期放行 73 % → 91 %、中途斷掉 3 → 1 次，等待 1.17 → 1.38 s
  （僅 6 台、待路測確認）。進入／離開記在 cloudlog：`ARS510 far stopped car on/off`。
- 畫面的「已送 radard」旗標改為：是二輪車點，或與 ACC／摘要點相同位置。
- 關閉物件清單（只跑 acc_summaries）：`ars510_radar_interface.py` 的 `STANDALONE_DISPLAY = False`；改回 fused：`PROFILE = PROFILES["fused"]`。
重播 route 5a（5 段 300 s 市區）：6000 次回報、無錯誤，點數 ACC 5397、摘要 3097、二輪車 124（10 個不同軌跡，多在右側 −2.9～−3.5 m 機車道）。

