/*
 * @Author: JohnJeep
 * @Date: 2025-03-31 19:48:52
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-03-31 20:00:52
 * @Description: 
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
 */
let x;
console.log(x, typeof x);

if (true) {
    let y = 1; // block scope
    var z = 2;  // z is hoisted to the top of the function or global scope
}
// console.log(y, typeof y); // ReferenceError: y is not defined
console.log(z, typeof z); // 2 "number", not suggest write like this


