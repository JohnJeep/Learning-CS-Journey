// Code generated by sqlc. DO NOT EDIT.
// versions:
//   sqlc v1.23.0

package db

import (
	"context"
	"database/sql"
)

type Querier interface {
	CreateAuthor(ctx context.Context, arg CreateAuthorParams) (sql.Result, error)
	DeleteAuthor(ctx context.Context, id int64) error
	GetAuthor(ctx context.Context, id int64) (Author, error)
	ListAuthors(ctx context.Context) ([]Author, error)
}

var _ Querier = (*Queries)(nil)