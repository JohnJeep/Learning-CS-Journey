import os
import shutil
from datetime import datetime

def get_tags_from_path(file_path, base_path):
    # Get relative path from base path
    rel_path = os.path.relpath(file_path, base_path)
    # Split path into components and remove filename
    path_parts = os.path.dirname(rel_path).split(os.sep)
    # Add filename without extension as last tag
    filename = os.path.splitext(os.path.basename(file_path))[0]
    path_parts.append(filename)
    # Convert to tags (lowercase first character)
    tags = [part[0].lower() + part[1:] if part else part for part in path_parts if part]
    return tags

def process_md_files(base_path, output_base):
    # Excluded directories
    excluded_dirs = {'Blogs', 'Embedded', 'Linux/vim/myvim'}
    
    # Create output directory if it doesn't exist
    os.makedirs(output_base, exist_ok=True)

    for root, dirs, files in os.walk(base_path):
        # Skip excluded directories
        rel_path = os.path.relpath(root, base_path)
        if any(rel_path.startswith(excluded) for excluded in excluded_dirs):
            continue

        for file in files:
            if file.endswith(".md"):
                file_path = os.path.join(root, file)
                
                # Calculate relative path to maintain directory structure
                output_dir = os.path.join(output_base, rel_path)
                os.makedirs(output_dir, exist_ok=True)
                output_path = os.path.join(output_dir, file)

                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
                
                # Skip if frontmatter already exists
                if content.startswith("---"):
                    continue
                
                # Extract title and category
                title = os.path.splitext(file)[0]
                category = os.path.basename(root)
                tags = get_tags_from_path(file_path, base_path)
                date = datetime.now().strftime("%Y/%m/%d %H:%M")
                
                # Create frontmatter with new format
                frontmatter = f"""---
title: {title}
date: {date}
updated: 
tags: {tags}
categories: {category}
---

"""
                # Write to new location with updated content
                with open(output_path, "w", encoding="utf-8") as f:
                    f.write(frontmatter + content)

if __name__ == "__main__":
    workspace_path = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(workspace_path, "Blogs", "source", "_posts")
    process_md_files(workspace_path, output_path)
