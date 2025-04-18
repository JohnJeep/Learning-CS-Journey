package ollama

import (
	"math"
	"testing"
)

func TestNormalize(t *testing.T) {
	testCases := []struct {
		name     string
		input    []float32
		expected []float32
	}{
		{
			name:     "Non-zero vector",
			input:    []float32{3, 4},
			expected: []float32{0.6, 0.8},
		},
		{
			name:     "Zero vector",
			input:    []float32{0, 0},
			expected: []float32{0, 0},
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			result := normalize(tc.input)
			if len(result) != len(tc.expected) {
				t.Fatalf("Expected length %d, got %d", len(tc.expected), len(result))
			}
			for i := range result {
				if math.Abs(float64(result[i]-tc.expected[i])) > 1e-9 {
					t.Errorf("At index %d, expected %f, got %f", i, tc.expected[i], result[i])
				}
			}
		})
	}
}
