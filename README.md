![](https://user-images.githubusercontent.com/47793918/233812617-beab2e71-57b9-479e-8bff-c3931347ca40.png)

## 🇹🇼 本分支：dimpleliang/openpilot `ars510-dev`

以 sunnypilot 為基礎，加上 ARS510 雷達與一系列針對 **2023 RAV4 Hybrid（台灣路況）** 的修改。個人分支，非 sunnypilot 官方版本。

**安裝網址（comma 3X 開機設定 → 安裝自訂軟體）：`installer.comma.ai/dimpleliang/ars510-dev`**

> ⚠️ **使用前請先了解**
> - 需要開啟 openpilot 縱向控制（Alpha）。開啟後原廠雷達被靜音，**原廠 PCS／AEB 會失效**，儀表會顯示 PCS 故障。
> - 轉角速度與扭力上限的放寬寫在 panda 韌體（已隨分支附上），安裝後會自動刷入；裝回原版 sunnypilot 會自動刷回原韌體。
> - 駕駛隨時要能接手，所有行為以實車紀錄調校，但仍是實驗性質。
> - 安裝與更新請在停車、熄火時進行。

### 主要修改

**雷達（ARS510）**
- 讀取 ARS510 雷達（不需 CAN filter）：控制用 ACC 目標＋最多 2 台 40 m 內摘要車；物件清單只用於畫面、機車與遠距判斷。
- 對向來車過濾、車道內機車優先當前車、遠距停止車（距離變化率交叉驗證）、雷達前車較穩定（減少與鏡頭來回切換）。
- 原廠 PCS 警報改從雷達匯流排讀取。

**縱向（加減速）**
- 遠處較慢的前車：提早收油、平順接近；遠距前車的煞車變柔和；雷達比鏡頭先看到時（120 m 外）先滑行。
- 加速較和緩：依車速的正加速度上限（倍率可調）、接近設定速度逐漸收斂、有前車時「跟速度、不追距離」。
- 高速無前車時補速（模型給 0 時不再卡住）、轉彎（方向盤 > 45°）不加速。
- 定速以儀表設定為準（儀表設 115 就跑到 115）。
- 停車更柔和：最後一段放鬆煞車、停住前預先放鬆、蠕行時煞車持續咬住、防止停住後前溜。
- 近距離停車保底：前車停住且在 20 m 內時，確保停在約 4.5 m 外。
- 偵測到車輛確實切入且比我慢時，短暫輕微收油（不加大煞車）。

**橫向（LTA 轉向）**
- 低速轉角速度放寬、70 km/h 以下扭力收力門檻放寬（路口、匝道轉得過）。
- 駕駛接手時命令跟隨實際角度、放手後漸進接回（不會突然回彈）；方向盤大角度快速回正時暫停 LTA，避免故障。

**畫面**
- 雷達前車下方顯示 `R 距離` 與車速。
- 預測切入的車：車上顯示琥珀色實心圓＋箭頭；真的切入且比我慢時圓圈閃爍。
- 國道事件橫幅（見下）。

**國道事件提醒（高公局即時事件）**
- 每 60 秒抓高公局 `LiveEvents.xml`，以 GPS＋內建國道路段線型判斷國道、方向、里程，只提示**同路、同方向、前方 3 km 內**的事件。
- 事故、故障車、散落物、施工、移動性施工、誤闖（機車／行人／動物／自行車）、逆向車、火災、濃霧、坑洞、積水、封閉等。
- 畫面橫幅（例「前方 1.4 km 故障車（外路肩）」）＋中文語音（「前方三公里、故障車」，1 km 再提醒一次）。
- 塞車時持續保留、隧道內以車速推算；誤闖類位置只是約略，顯示「前方約 2 km」。

**其他**
- 盲點偵測加速（每 0.2 秒輪詢盲點雷達，原廠只有 1 秒）。
- 修正鏡頭畫面約每秒閃黑一次。

**已移除**（為了給其他人使用）：自動上鎖／解鎖、熄火後車上自動產生報告。自動變道請用 sunnypilot 本身的設定。

### 個別關閉（在車機建立檔案即可，約 1 秒內生效）

| 檔案 | 作用 |
|---|---|
| `/data/ars510_no_latency_comp` | 關閉雷達物件 0.2 s 延遲補償 |
| `/data/ars510_no_cutin_marks` | 關閉畫面上的匯入／切入標示（預設只在接管時顯示） |
| `/data/hwyalert_off` | 關閉國道事件提醒 |
| `/data/hwyalert_voice_off` | 只關國道語音 |
| `/data/ars510_no_cutin_coast` | 關閉切入確認後的收油 |
| `/data/ars510_no_near_stop` | 關閉近距離停車保底 |
| `/data/ars510_no_far_lead_soft` | 關閉遠距前車柔和煞車與提早收油 |
| `/data/ars510_no_e2e_fill` | 關閉高速補速 |
| `/data/ars510_no_pre_release` | 關閉停住前預先放鬆煞車 |
| `/data/ars510_accel_scale` | 內容 0.3–1.0：加速上限倍率；0 = 不限制 |

### 分析工具（在電腦上對下載的 rlog 執行）

`tools/ars510/`：`drive_report.py`（整趟報告）、`merge_eval.py`／`merge_eval_slim.py`（切入標示重播評估）、
`hwyalert_replay.py`（國道提醒重播）、`slim_rlog.py`（縮小 rlog 方便傳輸）等。

完整修改紀錄、實測數據與參數說明：[ARS510_RADAR.md](ARS510_RADAR.md)

---

## 🌞 What is sunnypilot?
[sunnypilot](https://github.com/sunnyhaibin/sunnypilot) is a fork of comma.ai's openpilot, an open source driver assistance system. sunnypilot offers the user a unique driving experience for over 300+ supported car makes and models with modified behaviors of driving assist engagements. sunnypilot complies with comma.ai's safety rules as accurately as possible.

## 💭 Join our Community Forum
Join the official sunnypilot community forum to stay up to date with all the latest features and be a part of shaping the future of sunnypilot!
* https://community.sunnypilot.ai/

## Documentation
https://docs.sunnypilot.ai/ is your one stop shop for everything from features to installation to FAQ about the sunnypilot

## 🚘 Running on a dedicated device in a car
First, check out this list of items you'll need to [get started](https://community.sunnypilot.ai/t/getting-started-using-sunnypilot-in-your-supported-car/251).

## Installation
Next, refer to the sunnypilot community forum for [installation instructions](https://community.sunnypilot.ai/t/read-before-installing-sunnypilot/254), as well as a complete list of [Recommended Branch Installations](https://community.sunnypilot.ai/t/recommended-branch-installations/235).

## 🎆 Pull Requests
We welcome both pull requests and issues on GitHub. Bug fixes are encouraged.

Pull requests should be against the most current `master` branch.

## 📊 User Data

By default, sunnypilot uploads the driving data to comma servers. You can also access your data through [comma connect](https://connect.comma.ai/).

sunnypilot is open source software. The user is free to disable data collection if they wish to do so.

sunnypilot logs the road-facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera and microphone are only logged if you explicitly opt-in in settings.

By using this software, you understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

## Licensing

sunnypilot is released under the [MIT License](LICENSE). This repository includes original work as well as significant portions of code derived from [openpilot by comma.ai](https://github.com/commaai/openpilot), which is also released under the MIT license with additional disclaimers.

The original openpilot license notice, including comma.ai’s indemnification and alpha software disclaimer, is reproduced below as required:

> openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.
>
> Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.
>
> **THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
> YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
> NO WARRANTY EXPRESSED OR IMPLIED.**

For full license terms, please see the [`LICENSE`](LICENSE) file.

## 💰 Support sunnypilot
If you find any of the features useful, consider becoming a [sponsor on GitHub](https://github.com/sponsors/sunnyhaibin) to support future feature development and improvements.


By becoming a sponsor, you will gain access to exclusive content, early access to new features, and the opportunity to directly influence the project's development.


<h3>GitHub Sponsor</h3>

<a href="https://github.com/sponsors/sunnyhaibin">
  <img src="https://user-images.githubusercontent.com/47793918/244135584-9800acbd-69fd-4b2b-bec9-e5fa2d85c817.png" alt="Become a Sponsor" width="300" style="max-width: 100%; height: auto;">
</a>
<br>

<h3>PayPal</h3>

<a href="https://paypal.me/sunnyhaibin0850" target="_blank">
<img src="https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif" alt="PayPal this" title="PayPal - The safer, easier way to pay online!" border="0" />
</a>
<br></br>

Your continuous love and support are greatly appreciated! Enjoy 🥰

<span>-</span> Jason, Founder of sunnypilot
