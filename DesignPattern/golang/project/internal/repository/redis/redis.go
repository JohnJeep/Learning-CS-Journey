package redis

import (
	"fmt"
	"project/internal/interface/repositories"
	"project/internal/model"

	"github.com/go-redis/redis/v7"
)

type UserRepository struct {
	client *redis.Client
}

func New(client *redis.Client) repositories.UserRepository {
	return &UserRepository{
		client: client,
	}
}

func (r *UserRepository) GetByID(id int) (*model.User, error) {
	key := fmt.Sprintf("user:%d", id)
	result, err := r.client.HGetAll(key).Result()
	if err != nil {
		return nil, err
	}

	return &model.User{
		ID:    id,
		Name:  result["name"],
		Email: result["email"],
	}, nil
}

func (r *UserRepository) Save(user *model.User) error {
	key := fmt.Sprintf("user:%d", user.ID)
	return r.client.HSet(key,
		"name", user.Name,
		"email", user.Email,
	).Err()
}
