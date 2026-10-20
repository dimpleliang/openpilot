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
