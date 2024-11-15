// 日期时间格式格式：
// YYYY-MM-DD HH:mm:ss.SSS


// 获取当前时间
let currentDate = new Date();
let year = currentDate.getFullYear();
let month = currentDate.getMonth() + 1; // 注意：月份是从0开始计数的，所以要加1
let day = currentDate.getDate();
let hours = currentDate.getHours();
let minutes = currentDate.getMinutes();
let seconds = currentDate.getSeconds();

console.log("当前年份：", year);
console.log("当前月份：", month);
console.log("当前日期：", day);
console.log("当前小时：", hours);
console.log("当前分钟：", minutes);
console.log("当前秒钟：", seconds);

// 拼接成指定格式的时间字符串
let formattedTime = `${year}-${month}-${day} ${hours}:${minutes}:${seconds}`;
console.log(formattedTime);


// 获取当前时间戳。时间戳是指某个时刻距离 1970 年 1 月 1 日 00:00:00 UTC 的毫秒数。
let current = new Date();
let currentTimestamp = current.getTime();
console.log("当前时间的时间戳：", currentTimestamp);


function getCurrentISOTime() {
    let date = new Date();
    return date.toISOString();
}

function formatISOToChinaTime(isoString) {
    // 创建一个 Date 对象
    let date = new Date(isoString);
    // 将时间转换为 UTC+8，即中国标准时间
    date.setHours(date.getHours() + 8);

    // 获取年、月、日、时、分、秒、毫秒
    let year = date.getUTCFullYear();
    let month = String(date.getUTCMonth() + 1).padStart(2, '0');
    let day = String(date.getUTCDate()).padStart(2, '0');
    let hours = String(date.getUTCHours()).padStart(2, '0');
    let minutes = String(date.getUTCMinutes()).padStart(2, '0');
    let seconds = String(date.getUTCSeconds()).padStart(2, '0');
    let milliseconds = String(date.getUTCMilliseconds()).padStart(3, '0');

    // 格式化为 YYYY-MM-DD HH:mm:ss.SSS
    return `${year}-${month}-${day} ${hours}:${minutes}:${seconds}.${milliseconds}`;
}