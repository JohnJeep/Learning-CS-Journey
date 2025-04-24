package mysql

import (
	"project/internal/model"
	"testing"

	"github.com/DATA-DOG/go-sqlmock"
	"github.com/stretchr/testify/assert"
)

func TestUserRepository_GetByID(t *testing.T) {
	// 创建模拟数据库连接
	db, mock, err := sqlmock.New()
	if err != nil {
		t.Fatalf("an error '%s' was not expected", err)
	}
	defer db.Close()

	// 创建Repository实例
	repo := New(db)

	// 测试数据
	rows := sqlmock.NewRows([]string{"id", "name", "email"}).
		AddRow(1, "Test User", "test@example.com")

	// 设置预期查询
	mock.ExpectQuery("SELECT id, name, email FROM users WHERE id = ?").
		WithArgs(1).
		WillReturnRows(rows)

	// 执行测试
	user, err := repo.GetByID(1)

	// 验证结果
	assert.NoError(t, err)
	assert.Equal(t, 1, user.ID)
	assert.Equal(t, "Test User", user.Name)

	// 确保所有预期都满足
	assert.NoError(t, mock.ExpectationsWereMet())
}

func TestUserRepository_Save(t *testing.T) {
	db, mock, err := sqlmock.New()
	if err != nil {
		t.Fatalf("an error '%s' was not expected", err)
	}
	defer db.Close()

	repo := New(db)

	user := &model.User{
		Name:  "New User",
		Email: "new@example.com",
	}

	mock.ExpectExec("INSERT INTO users \\(name, email\\) VALUES \\(\\?, \\?\\)").
		WithArgs(user.Name, user.Email).
		WillReturnResult(sqlmock.NewResult(1, 1))

	err = repo.Save(user)

	assert.NoError(t, err)
	assert.NoError(t, mock.ExpectationsWereMet())
}
