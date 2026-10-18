# ARS510 雷達整合（RAV4 2022-25 / C-HR 2021-22 / Yaris）

本分支 = sunnypilot `release-tizi`（v2026.002.002，comma 3X）＋ [ds-sebastian/ars510-radar](https://github.com/ds-sebastian/ars510-radar)
安裝器（commit 7ff9485，2026-10-18）以預設 `acc` profile 套用後的結果，對應上游提案 [commaai/opendbc#3860](https://github.com/commaai/opendbc/pull/3860)。

- 不需要 CAN filter 即可讀取雷達；但開啟 openpilot 縱向控制（Alpha）時雷達會被靜音，原廠 PCS/AEB 失效，儀表會顯示 PCS 故障。
- 變更只在 `opendbc_repo/opendbc/car/toyota/`（新增 ars510 套件與 interface.py 結尾的 hook 區塊）與 `opendbc_repo/opendbc/dbc/ars510_acc_target.dbc`，其餘與 release-tizi 相同。
- 裝置安裝網址：`https://installer.comma.ai/dimpleliang/release-tizi-ars510`
- 要還原成純視覺：改裝回 `release.sunnypilot.ai`（release-tizi）。
- 更新做法：以上游 release-tizi 重新套用安裝器後 force push 本分支。
