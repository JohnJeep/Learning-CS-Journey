package service

import (
	"errors"
	"project/internal/model"
	mocks "project/mock/repositories"
	"testing"

	"github.com/stretchr/testify/assert"
	"go.uber.org/mock/gomock"
)

func TestUserService_GetUser_Success(t *testing.T) {
	ctrl := gomock.NewController(t)
	defer ctrl.Finish()

	// 创建Mock仓库
	mockRepo := mocks.NewMockUserRepository(ctrl)

	// 设置预期行为
	expectedUser := &model.User{
		ID:    1,
		Name:  "Test User",
		Email: "test@example.com",
	}

	mockRepo.EXPECT().
		GetByID(1).
		Return(expectedUser, nil).
		Times(1)

	// 创建服务并注入Mock
	service := New(mockRepo)

	// 执行测试
	user, err := service.GetUser(1)

	// 验证结果
	assert.NoError(t, err)
	assert.Equal(t, expectedUser, user)
}

func TestUserService_CreateUser_Error(t *testing.T) {
	ctrl := gomock.NewController(t)
	defer ctrl.Finish()

	mockRepo := mocks.NewMockUserRepository(ctrl)

	// 设置预期返回错误
	testErr := errors.New("database error")
	mockRepo.EXPECT().
		Save(gomock.Any()).
		Return(testErr).
		Times(1)

	service := New(mockRepo)
	err := service.CreateUser("Fail User", "fail@example.com")

	assert.Error(t, err)
	assert.Equal(t, testErr, err)
}
