package common

const (
	RegisterAction     = "REGISTER"
	CreateDeviceAction = "CREATE_DEVICE"
	UpdateDeviceAction = "UPDATE_DEVICE"
	HeartbeatAction    = "HEARTBEAT"
)

type Message struct {
	Action  string      `json:"action"`
	Code    string      `json:"code,omitempty"`
	Payload interface{} `json:"payload,omitempty"`
}

type Response struct {
	Status  int         `json:"status"`
	Message string      `json:"message"`
	Data    interface{} `json:"data,omitempty"`
}
