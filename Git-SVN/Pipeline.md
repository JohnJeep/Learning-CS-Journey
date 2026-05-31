<!--
 * @Author: JohnJeep
 * @Date: 2019-04-04 23:29:59
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 18:37:17
 * @Description: Git Pipeline
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

## Git Pipeline

Gitlab Pipeline Rules

"Pipeline must succeed"：合并前 Pipeline 必须通过

"Require approvals before merging"：合并需要审批

一旦 MR 有审批记录，后续修改可自由触发 Pipeline。
修改后提交 → 自动触发 Pipeline → 无需重新审批