package demo

import "testing"

func TestAdd(t *testing.T) {
	type args struct {
		x int
		y int
	}
	tests := []struct {
		name string
		args args
		want int
	}{
		{
			name: "add_1",
			args: args{
				x: 10,
				y: 20,
			},
			want: 30,
		},
		{
			name: "add_2",
			args: args{
				x: -2,
				y: -10,
			},
			want: -11,
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			if got := Add(tt.args.x, tt.args.y); got != tt.want {
				t.Errorf("Add() = %v, want %v", got, tt.want)
			}
		})
	}
}
