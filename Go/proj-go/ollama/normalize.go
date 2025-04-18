/*
 * @Author: JohnJeep
 * @Date: 2025-04-18 17:34:48
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-18 17:36:33
 * @Description: 对输入的浮点数切片 vec 进行归一化处理。
 *               计算向量的模，然后将向量中的每个元素除以该模，将向量转换为单位向量。
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package ollama

import "math"

func normalize(vec []float32) []float32 {
	var sum float32
	for _, v := range vec {
		sum += v * v
	}

	norm := float32(0.0)
	if sum > 0 {
		norm = float32(1.0 / math.Sqrt(float64(sum)))
	}

	for i := range vec {
		vec[i] *= norm
	}
	return vec
}
