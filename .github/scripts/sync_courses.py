#!/usr/bin/env python3
"""
Sync course metadata from ES-courses yaml files to excursion-studio.github.io courses.json
Rebuilds sections from yaml files, preserves other JSON structure.
"""

import yaml
import json
import argparse
from pathlib import Path
from collections import defaultdict


def load_yaml(filepath):
    """Load YAML file"""
    with open(filepath, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def load_json(filepath):
    """Load JSON file"""
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)


def save_json(filepath, data):
    """Save JSON file with pretty formatting"""
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def find_course_yaml_files(courses_dir):
    """Find all course yaml files in the courses directory"""
    courses_path = Path(courses_dir)
    yaml_files = []
    
    for course_dir in courses_path.iterdir():
        if course_dir.is_dir():
            for yaml_file in course_dir.glob('*.yaml'):
                yaml_files.append(yaml_file)
    
    return yaml_files


def build_course_item(course_data, lang):
    """Build a course item for the JSON structure"""
    lang_data = course_data.get(lang, {})
    
    return {
        'id': course_data.get('id', ''),
        'title': lang_data.get('title', ''),
        'text': lang_data.get('text', ''),
        'topics': lang_data.get('topics', []),
        'tags': lang_data.get('tags', []),
        'link': course_data.get('link', ''),
        'available': course_data.get('available', False)
    }


def update_courses_json(courses_dir, output_dir):
    """Update courses.json files for both languages"""
    courses_path = Path(courses_dir)
    yaml_files = find_course_yaml_files(courses_dir)
    
    courses_by_section = defaultdict(list)
    section_titles_by_section = {}
    
    for yaml_file in yaml_files:
        course_data = load_yaml(yaml_file)
        course_id = course_data.get('id', '')
        if course_id:
            section_id = course_data.get('section', '')
            if section_id:
                courses_by_section[section_id].append(course_data)
                for lang in ['en', 'zh']:
                    lang_data = course_data.get(lang, {})
                    section_title = lang_data.get('sectionTitle', '')
                    if section_title:
                        if section_id not in section_titles_by_section:
                            section_titles_by_section[section_id] = {}
                        section_titles_by_section[section_id][lang] = section_title
    
    section_order = sorted(courses_by_section.keys())
    print(f"Section order (alphabetical): {section_order}")
    
    continue_yaml_path = courses_path / 'continue.yaml'
    continue_data = None
    if continue_yaml_path.exists():
        continue_data = load_yaml(continue_yaml_path)
        print(f"Loaded continue.yaml")
    
    for lang in ['en', 'zh']:
        output_path = Path(output_dir) / 'data' / lang / 'courses.json'
        
        if not output_path.exists():
            print(f"Warning: {output_path} does not exist, skipping")
            continue
        
        existing_data = load_json(output_path)
        
        new_sections = []
        for section_id in section_order:
            items = []
            for course_data in sorted(courses_by_section[section_id], key=lambda x: x.get('id', '')):
                items.append(build_course_item(course_data, lang))
            
            section_title = section_titles_by_section.get(section_id, {}).get(lang, section_id)
            new_sections.append({
                'type': 'courses',
                'title': section_title,
                'items': items
            })
            print(f"Built section '{section_id}' with {len(items)} items in {lang}/courses.json")
        
        existing_data['sections'] = new_sections
        
        if continue_data and lang in continue_data:
            lang_continue = continue_data[lang]
            if 'continueCard' not in existing_data:
                existing_data['continueCard'] = {}
            existing_data['continueCard']['futureTags'] = lang_continue.get('futureTags', existing_data['continueCard'].get('futureTags', []))
            print(f"Updated continueCard.futureTags in {lang}/courses.json")
        
        save_json(output_path, existing_data)
        print(f"Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Sync course metadata from yaml files to courses.json')
    parser.add_argument('--courses-dir', required=True, help='Path to ES-courses directory')
    parser.add_argument('--output-dir', required=True, help='Path to excursion-studio.github.io directory')
    
    args = parser.parse_args()
    update_courses_json(args.courses_dir, args.output_dir)
    print("Sync completed successfully!")


if __name__ == '__main__':
    main()
