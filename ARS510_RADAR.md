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
