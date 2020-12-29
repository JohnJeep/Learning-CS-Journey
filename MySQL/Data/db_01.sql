/*
 Navicat Premium Data Transfer

 Source Server         : CentOS7_root
 Source Server Type    : MySQL
 Source Server Version : 50731
 Source Host           : 192.168.1.98:3306
 Source Schema         : db_01

 Target Server Type    : MySQL
 Target Server Version : 50731
 File Encoding         : 65001

 Date: 29/12/2020 23:25:11
*/

SET NAMES utf8mb4;
SET FOREIGN_KEY_CHECKS = 0;

-- ----------------------------
-- Table structure for article
-- ----------------------------
DROP TABLE IF EXISTS `article`;
CREATE TABLE `article`  (
  `id` int(10) UNSIGNED NOT NULL AUTO_INCREMENT,
  `author_id` int(10) UNSIGNED NOT NULL,
  `category_id` int(10) UNSIGNED NOT NULL,
  `views` int(10) UNSIGNED NOT NULL,
  `comments` int(10) UNSIGNED NOT NULL,
  `title` varchar(255) CHARACTER SET utf8 COLLATE utf8_general_ci NOT NULL,
  `content` text CHARACTER SET utf8 COLLATE utf8_general_ci NOT NULL,
  PRIMARY KEY (`id`) USING BTREE
) ENGINE = InnoDB AUTO_INCREMENT = 4 CHARACTER SET = utf8 COLLATE = utf8_general_ci ROW_FORMAT = Dynamic;

-- ----------------------------
-- Records of article
-- ----------------------------
INSERT INTO `article` VALUES (1, 1, 1, 1, 1, '1', '1');
INSERT INTO `article` VALUES (2, 2, 2, 2, 2, '2', '2');
INSERT INTO `article` VALUES (3, 1, 1, 3, 3, '3', '3');

-- ----------------------------
-- Table structure for book
-- ----------------------------
DROP TABLE IF EXISTS `book`;
CREATE TABLE `book`  (
  `bookid` int(10) UNSIGNED NOT NULL AUTO_INCREMENT,
  `card` int(10) UNSIGNED NOT NULL,
  PRIMARY KEY (`bookid`) USING BTREE
) ENGINE = InnoDB AUTO_INCREMENT = 20 CHARACTER SET = utf8 COLLATE = utf8_general_ci ROW_FORMAT = Dynamic;

-- ----------------------------
-- Records of book
-- ----------------------------
INSERT INTO `book` VALUES (1, 9);
INSERT INTO `book` VALUES (2, 9);
INSERT INTO `book` VALUES (3, 17);
INSERT INTO `book` VALUES (4, 18);
INSERT INTO `book` VALUES (5, 19);
INSERT INTO `book` VALUES (6, 19);
INSERT INTO `book` VALUES (7, 19);
INSERT INTO `book` VALUES (8, 18);
INSERT INTO `book` VALUES (9, 13);
INSERT INTO `book` VALUES (10, 8);
INSERT INTO `book` VALUES (11, 4);
INSERT INTO `book` VALUES (12, 13);
INSERT INTO `book` VALUES (13, 12);
INSERT INTO `book` VALUES (14, 20);
INSERT INTO `book` VALUES (15, 4);
INSERT INTO `book` VALUES (16, 2);
INSERT INTO `book` VALUES (17, 15);
INSERT INTO `book` VALUES (18, 8);
INSERT INTO `book` VALUES (19, 13);

-- ----------------------------
-- Table structure for class
-- ----------------------------
DROP TABLE IF EXISTS `class`;
CREATE TABLE `class`  (
  `id` int(10) UNSIGNED NOT NULL AUTO_INCREMENT,
  `card` int(10) UNSIGNED NOT NULL,
  PRIMARY KEY (`id`) USING BTREE
) ENGINE = InnoDB AUTO_INCREMENT = 22 CHARACTER SET = utf8 COLLATE = utf8_general_ci ROW_FORMAT = Dynamic;

-- ----------------------------
-- Records of class
-- ----------------------------
INSERT INTO `class` VALUES (1, 13);
INSERT INTO `class` VALUES (2, 2);
INSERT INTO `class` VALUES (3, 10);
INSERT INTO `class` VALUES (4, 5);
INSERT INTO `class` VALUES (5, 13);
INSERT INTO `class` VALUES (6, 9);
INSERT INTO `class` VALUES (7, 6);
INSERT INTO `class` VALUES (8, 2);
INSERT INTO `class` VALUES (9, 12);
INSERT INTO `class` VALUES (10, 15);
INSERT INTO `class` VALUES (11, 19);
INSERT INTO `class` VALUES (12, 8);
INSERT INTO `class` VALUES (13, 4);
INSERT INTO `class` VALUES (14, 13);
INSERT INTO `class` VALUES (15, 14);
INSERT INTO `class` VALUES (16, 11);
INSERT INTO `class` VALUES (17, 13);
INSERT INTO `class` VALUES (18, 10);
INSERT INTO `class` VALUES (19, 9);
INSERT INTO `class` VALUES (20, 16);
INSERT INTO `class` VALUES (21, 11);

-- ----------------------------
-- Table structure for computer
-- ----------------------------
DROP TABLE IF EXISTS `computer`;
CREATE TABLE `computer`  (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `name` varchar(20) CHARACTER SET utf8 COLLATE utf8_general_ci NULL DEFAULT NULL,
  PRIMARY KEY (`id`) USING BTREE
) ENGINE = InnoDB AUTO_INCREMENT = 4 CHARACTER SET = utf8 COLLATE = utf8_general_ci ROW_FORMAT = Dynamic;

-- ----------------------------
-- Records of computer
-- ----------------------------
INSERT INTO `computer` VALUES (1, 'dell');
INSERT INTO `computer` VALUES (2, 'ASUA');
INSERT INTO `computer` VALUES (3, '苹果');

-- ----------------------------
-- Table structure for tbl_dept
-- ----------------------------
DROP TABLE IF EXISTS `tbl_dept`;
CREATE TABLE `tbl_dept`  (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `name` varchar(30) CHARACTER SET utf8 COLLATE utf8_general_ci NULL DEFAULT NULL,
  `locAdd` varchar(40) CHARACTER SET utf8 COLLATE utf8_general_ci NULL DEFAULT NULL,
  PRIMARY KEY (`id`) USING BTREE
) ENGINE = InnoDB AUTO_INCREMENT = 6 CHARACTER SET = utf8 COLLATE = utf8_general_ci ROW_FORMAT = Dynamic;

-- ----------------------------
-- Records of tbl_dept
-- ----------------------------
INSERT INTO `tbl_dept` VALUES (1, 'RD', '11');
INSERT INTO `tbl_dept` VALUES (2, 'HR', '12');
INSERT INTO `tbl_dept` VALUES (3, 'NK', '13');
INSERT INTO `tbl_dept` VALUES (4, 'MIS', '14');
INSERT INTO `tbl_dept` VALUES (5, 'FD', '15');

-- ----------------------------
-- Table structure for tbl_employee
-- ----------------------------
DROP TABLE IF EXISTS `tbl_employee`;
CREATE TABLE `tbl_employee`  (
  `id` int(11) NOT NULL AUTO_INCREMENT,
  `name` varchar(20) CHARACTER SET utf8 COLLATE utf8_general_ci NULL DEFAULT NULL,
  `deptId` int(11) NULL DEFAULT NULL,
  PRIMARY KEY (`id`) USING BTREE
) ENGINE = InnoDB AUTO_INCREMENT = 9 CHARACTER SET = utf8 COLLATE = utf8_general_ci ROW_FORMAT = Dynamic;

-- ----------------------------
-- Records of tbl_employee
-- ----------------------------
INSERT INTO `tbl_employee` VALUES (1, 'z3', 1);
INSERT INTO `tbl_employee` VALUES (2, 'z4', 1);
INSERT INTO `tbl_employee` VALUES (3, 'z5', 1);
INSERT INTO `tbl_employee` VALUES (4, 'w5', 2);
INSERT INTO `tbl_employee` VALUES (5, 'w6', 2);
INSERT INTO `tbl_employee` VALUES (6, 's7', 3);
INSERT INTO `tbl_employee` VALUES (7, 's8', 4);
INSERT INTO `tbl_employee` VALUES (8, 's9', 51);

SET FOREIGN_KEY_CHECKS = 1;
