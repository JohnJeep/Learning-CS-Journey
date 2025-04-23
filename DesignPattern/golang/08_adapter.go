/*
 * @Author: JohnJeep
 * @Date: 2025-04-21 17:22:55
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-23 18:26:01
 * @Description: adpater pattern
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import "fmt"

type AudioPlayer interface {
	Play(audioType string, fileName string)
}

type MP3Player struct{}

func (m *MP3Player) Play(audioType string, fileName string) {
	if audioType == "mp3" {
		fmt.Println("Playing MP3 file. Name:", fileName)
	} else {
		fmt.Println("Invalid audio type for MP3 player. Supported type: mp3")
	}
}

type WAVPlayer struct{}

func (w *WAVPlayer) Play(audioType string, fileName string) {
	if audioType == "wav" {
		fmt.Println("Playing WAV file. Name:", fileName)
	} else {
		fmt.Println("Invalid audio type for WAV player. Supported type: wav")
	}
}

// WAVPlayerAdapter is an adapter for WAVPlayer
type WAVPlayerAdapter struct {
	wavPlayer *WAVPlayer
}

func NewWAVPlayerAdapter() *WAVPlayerAdapter {
	return &WAVPlayerAdapter{
		wavPlayer: &WAVPlayer{},
	}
}

// Play method for WAVPlayerAdapter
// It adapts the WAVPlayer to the AudioPlayer interface
// and allows it to play WAV files
func (w *WAVPlayerAdapter) Play(audioType string, fileName string) {
	if audioType == "wav" {
		w.wavPlayer.Play(audioType, fileName)
	} else {
		fmt.Println("Invalid audio type for WAV player adapter. Supported type: wav")
	}
}

// AdvancedAudioPlayer is an adapter that allows playing both MP3 and WAV files
// It implements the AudioPlayer interface
type AdvancedAudioPlayer struct {
	mp3Player        *MP3Player
	wavPlayerAdapter *WAVPlayerAdapter
}

func NewAdvancedAudioPlayer() *AdvancedAudioPlayer {
	return &AdvancedAudioPlayer{
		mp3Player:        &MP3Player{},
		wavPlayerAdapter: NewWAVPlayerAdapter(),
	}
}

// Play method for AdvancedAudioPlayer
// It checks the audio type and delegates the play request to the appropriate player
func (a *AdvancedAudioPlayer) Play(audioType string, fileName string) {
	if audioType == "mp3" {
		a.mp3Player.Play(audioType, fileName)
	} else if audioType == "wav" {
		a.wavPlayerAdapter.Play(audioType, fileName)
	} else {
		fmt.Println("Invalid audio type for audio player. Supported types: mp3, wav")
	}
}

func main() {
	fmt.Println("Adapter Pattern")
	audioPlayer := NewAdvancedAudioPlayer()
	audioPlayer.Play("mp3", "song.mp3")
	audioPlayer.Play("wav", "song.wav")
	audioPlayer.Play("flac", "song.flac")
	audioPlayer.Play("mp4", "song.mp4")
}
