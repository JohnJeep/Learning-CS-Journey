package redis

import (
	"project/internal/model"
	"testing"

	"github.com/alicebob/miniredis/v2"
	"github.com/go-redis/redis/v7"
	"github.com/stretchr/testify/assert"
)

func TestRedisUserRepository(t *testing.T) {
	// 启动迷你Redis服务器
	mr, err := miniredis.Run()
	if err != nil {
		t.Fatal(err)
	}
	defer mr.Close()

	// 创建Redis客户端
	client := redis.NewClient(&redis.Options{
		Addr: mr.Addr(),
	})

	repo := New(client)

	t.Run("Save and Get User", func(t *testing.T) {
		user := &model.User{
			ID:    1,
			Name:  "Redis User",
			Email: "redis@example.com",
		}

		// 测试Save
		err := repo.Save(user)
		assert.NoError(t, err)

		// 测试GetByID
		result, err := repo.GetByID(1)
		assert.NoError(t, err)
		assert.Equal(t, user.Name, result.Name)
		assert.Equal(t, user.Email, result.Email)
	})
}
