/*
 * @Author: JohnJeep
 * @Date: 2025-04-21 17:22:55
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-24 11:20:31
 * @Description: adpater pattern
 *               1. 新增播放器支持 AudioPlayer 接口，直接实现接口，注册到 factory 中
 *               2. 新增播放器不兼容 AudioPlayer 接口，使用适配器模式，将其转换为 AudioPlayer 接口。
 *                  在注册到 factory 中。
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import (
	"errors"
	"fmt"
)

type AudioPlayer interface {
	Play(fileName string)
}

type MP3Player struct{}

func (m *MP3Player) Play(fileName string) {
	fmt.Println("Playing MP3 file. Name:", fileName)
}

type WAVPlayer struct{}

func (w *WAVPlayer) Play(fileName string) {
	fmt.Println("Playing WAV file. Name:", fileName)
}

// FLACDecoder is a third-party library for decoding FLAC files,
// it does not implement the AudioPlayer interface,
// so we need to create an adapter for it.
type FLACDecoder struct{}

func (f *FLACDecoder) Decode(fileName string) {
	fmt.Println("Decoding FLAC file. Name:", fileName)
}

// FLACAdapter is an adapter for FLAC files
type FLACAdapter struct {
	decoder *FLACDecoder
}

// FLAC format is not supported directly, so we need to convert it to existing format
func (f *FLACAdapter) Play(fileName string) {
	f.decoder.Decode(fileName)
	fmt.Println("Playing FLAC file (converted to WAV):", fileName)
}

// PlayerFactory is a factory for creating audio players
type PlayerFactory struct {
	players map[string]AudioPlayer // interface as a field
}

func NewPlayerFactory() *PlayerFactory {
	pf := &PlayerFactory{
		players: make(map[string]AudioPlayer),
	}

	// add new player, register it, no need to modify existing code
	pf.Register("mp3", &MP3Player{})
	pf.Register("wav", &WAVPlayer{})
	pf.Register("flac", &FLACAdapter{decoder: &FLACDecoder{}})

	return pf
}

// register a new apapter
// AudioPlayer interface as a parameter
func (p *PlayerFactory) Register(audioType string, player AudioPlayer) {
	p.players[audioType] = player
}

// interface as a return value
func (p *PlayerFactory) GetPlayer(audioType string) (AudioPlayer, error) {
	player, exists := p.players[audioType]
	if !exists {
		return nil, errors.New("unsupport audio format " + audioType)
	}
	return player, nil
}

func main() {
	fmt.Println("Adapter Pattern")
	f := NewPlayerFactory()

	testFiles := []struct {
		format string
		file   string
	}{
		{"mp3", "song.mp3"},
		{"wav", "audio.wav"},
		{"flac", "music.flac"},
		{"mp4", "video.mp4"},
		{"avi", "movie.avi"},
	}
	for _, testFile := range testFiles {
		player, err := f.GetPlayer(testFile.format)
		if err != nil {
			fmt.Println("Error:", err)
			continue
		}
		player.Play(testFile.file)
	}
}
