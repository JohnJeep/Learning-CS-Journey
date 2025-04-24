package mysql

import (
	"database/sql"
	"project/internal/interface/repositories"
	"project/internal/model"
)

type UserRepository struct {
	db *sql.DB
}

func New(db *sql.DB) repositories.UserRepository {
	return &UserRepository{db: db}
}

func (r *UserRepository) GetByID(id int) (*model.User, error) {
	user := &model.User{}
	err := r.db.QueryRow(
		"SELECT id, name, email FROM users WHERE id = ?",
		id,
	).Scan(&user.ID, &user.Name, &user.Email)

	return user, err
}

func (r *UserRepository) Save(user *model.User) error {
	_, err := r.db.Exec(
		"INSERT INTO users (name, email) VALUES (?, ?)",
		user.Name, user.Email,
	)
	return err
}
