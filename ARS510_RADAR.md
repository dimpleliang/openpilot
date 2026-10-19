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
不影響 radard 選前車。剛出現、追不到 1.2 秒的目標畫成半透明、只標距離（距離與速度尚未收斂）。已經有雷達前車箭頭的那台不重複標示。只影響畫面，radarTracks 與控制邏輯不變。

## 發佈門檻（控制）

`fused` 的 `min_publish_age` 由上游的 60 週期（約 3.6 秒）改為 20 週期（約 1.2 秒），雷達第一次看到就在路徑上的車可更早用於控制。
仍保留 Kalman 速度收斂檢查（publish_speed_std_mps 0.75 m/s）；一般車速下 radard 另以鏡頭前車核對距離（25 % / 5 m）與速度。
剩餘風險在低速（約 4 m/s 以下）：radard 不經鏡頭確認、直接採用正前方 25 m 內最近的雷達點。
