package repositories

import "project/internal/model"

type UserRepository interface {
	GetByID(id int) (*model.User, error)
	Save(user *model.User) error
}
