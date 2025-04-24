/*
 * @Author: JohnJeep
 * @Date: 2025-04-24 09:52:09
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-24 10:34:53
 * @Description: power adapter
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */

package main

import (
	"errors"
	"fmt"
)

type PowerAdapter interface {
	Convert() (float64, error)
}

type ChinaPowerSupply struct{}

func (c *ChinaPowerSupply) InputVoltage() (float64, error) {
	return 220.0, nil
}

type baseAdapter struct {
	source *ChinaPowerSupply
}

func newBaseAdapter() *baseAdapter {
	return &baseAdapter{
		source: &ChinaPowerSupply{},
	}
}

// 3.3v adapter
type Adapter3V3 struct {
	*baseAdapter
}

// 5V adapter
type Adapter5V struct {
	*baseAdapter
}

// 12V adapter
type Adapter12V struct {
	*baseAdapter
}

// 24V adapter
type Adapter24V struct {
	*baseAdapter
}

func (a *Adapter3V3) Convert() (float64, error) {
	// Convert to 3.3V
	voltage, err := a.source.InputVoltage()
	if err != nil {
		return 0, err
	}
	return voltage * 0.015, nil
}

func (a *Adapter5V) Convert() (float64, error) {
	// Convert to 5V
	voltage, err := a.source.InputVoltage()
	if err != nil {
		return 0, err
	}
	return voltage * 0.0227, nil
}

func (a *Adapter12V) Convert() (float64, error) {
	// Convert to 12V
	voltage, err := a.source.InputVoltage()
	if err != nil {
		return 0, err
	}
	return voltage * 0.0545, nil
}

func (a *Adapter24V) Convert() (float64, error) {
	// Convert to 24V
	voltage, err := a.source.InputVoltage()
	if err != nil {
		return 0, err
	}
	return voltage * 0.1091, nil
}

type VoltageType int

const (
	V3V3 VoltageType = iota
	V5V
	V12V
	V24V
)

// this is a factory function
// it returns a PowerAdapter based on the voltage type
// PowerAdpter is an interface type, implicit polymorphic calls
func PowerAdapterFactory(v VoltageType) (PowerAdapter, error) {
	switch v {
	case V3V3:
		return &Adapter3V3{baseAdapter: newBaseAdapter()}, nil
	case V5V:
		return &Adapter5V{baseAdapter: newBaseAdapter()}, nil
	case V12V:
		return &Adapter12V{baseAdapter: newBaseAdapter()}, nil
	case V24V:
		return &Adapter24V{baseAdapter: newBaseAdapter()}, nil
	default:
		return nil, errors.New("unsupported voltage type")
	}
}

func main() {
	voltages := []VoltageType{V3V3, V5V, V12V, V24V}
	for _, v := range voltages {
		adapter, err := PowerAdapterFactory(v)
		if err != nil {
			fmt.Println("Error:", err)
			continue
		}
		voltage, err := adapter.Convert() // Convert to the desired voltage, dynamic call
		if err != nil {
			fmt.Println("Error converting voltage:", err)
			continue
		}
		fmt.Printf("output voltage:%.1fV\n", voltage)
	}
}
