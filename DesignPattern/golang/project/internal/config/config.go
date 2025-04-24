package config

type DatabaseConfig struct {
	Type     string
	Host     string
	Port     int
	Username string
	Password string
	Database string
}

func Load() *DatabaseConfig {
	return &DatabaseConfig{
		Type:     "mysql",
		Host:     "",
		Port:     3306,
		Username: "mysqladmin",
		Password: "Mysql2020Admin",
		Database: "iotplat",
	}
}
