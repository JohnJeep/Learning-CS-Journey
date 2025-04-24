package service

import (
	"project/internal/interface/repositories"
	"project/internal/model"
)

type UserService struct {
	repo repositories.UserRepository
}

func New(repo repositories.UserRepository) *UserService {
	return &UserService{repo: repo}
}

func (s *UserService) GetUser(id int) (*model.User, error) {
	return s.repo.GetByID(id)
}

func (s *UserService) CreateUser(name, email string) error {
	user := &model.User{
		Name:  name,
		Email: email,
	}
	return s.repo.Save(user)
}
