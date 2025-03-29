import os
from datetime import datetime

def add_frontmatter_to_md_files(base_path):
    for root, _, files in os.walk(base_path):
        for file in files:
            if file.endswith(".md"):
                file_path = os.path.join(root, file)
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read()
                
                # Skip if frontmatter already exists
                if content.startswith("---"):
                    continue
                
                # Extract title, category, and tags
                title = os.path.splitext(file)[0]
                category = os.path.basename(root)
                tags = [category]  # Use the directory name as the tag
                date = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                # Create frontmatter
                frontmatter = f"""---
title: {title}
data: {date}
tags: {tags}
category: {category}
---

"""
                # Write updated content back to the file
                with open(file_path, "w", encoding="utf-8") as f:
                    f.write(frontmatter + content)

if __name__ == "__main__":
    workspace_path = os.path.dirname(os.path.abspath(__file__))  # Dynamically set to current script's directory
    add_frontmatter_to_md_files(workspace_path)
