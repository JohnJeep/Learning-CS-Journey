#!/bin/bash
# 用法: ./format_md.sh [文件或目录路径]（默认为当前目录）

process_file() {
    local file="$1"
    echo "正在处理: $file"
    
    awk '
    BEGIN {
        in_code_block = 0
        is_header = 0
    }
    
    # 检测代码块
    /^```|^~~~/ { in_code_block = !in_code_block }
    
    # 代码块内容原样输出
    in_code_block { 
        print
        is_header = 0
        next 
    }
    
    # 处理标题行
    /^#+ / {
        # 确保标题前有2空行（文件开头除外）
        if (NR > 1 && !is_header) {
            print "\n"
        }
        
        # 打印标题行
        print
        is_header = 1
        next
    }
    
    # 其他内容保持原样
    {
        # 重置标题标记
        is_header = 0
        print
    }
    ' "$file" > "${file}.tmp" && mv "${file}.tmp" "$file"
}

target="${1:-.}"

if [[ -f "$target" && "$target" == *.md ]]; then
    echo "处理单个文件: $target"
    process_file "$target"
elif [[ -d "$target" ]]; then
    echo "处理目录: $target"
    find "$target" -type f -name "*.md" | while read -r file; do
        process_file "$file"
    done
else
    echo "错误: 输入必须是 .md 文件或目录"
    exit 1
fi

echo "格式化完成！"