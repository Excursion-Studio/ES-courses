@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

echo ========================================
echo        ES-courses 课程创建工具
echo ========================================
echo.

set /p "COURSE_NAME_ZH=请输入课程中文名称: "
if "!COURSE_NAME_ZH!"=="" (
    echo 错误: 课程中文名称不能为空
    pause
    exit /b 1
)

set /p "COURSE_NAME_EN=请输入课程英文名称: "
if "!COURSE_NAME_EN!"=="" (
    echo 错误: 课程英文名称不能为空
    pause
    exit /b 1
)

set /p "SECTION_NAME_ZH=请输入系列中文名称: "
if "!SECTION_NAME_ZH!"=="" (
    echo 错误: 系列中文名称不能为空
    pause
    exit /b 1
)

set /p "SECTION_NAME_EN=请输入系列英文名称: "
if "!SECTION_NAME_EN!"=="" (
    echo 错误: 系列英文名称不能为空
    pause
    exit /b 1
)

set "COURSE_ID=!COURSE_NAME_EN!"
set "COURSE_ID=!COURSE_ID: =-!"
set "COURSE_ID=!COURSE_ID:.=-!"
call :toLower COURSE_ID

set "SECTION_ID=!SECTION_NAME_EN!"
set "SECTION_ID=!SECTION_ID: =-!"
call :toLower SECTION_ID

set "FOLDER_NAME=!COURSE_ID!"
set "BASE_DIR=%~dp0"
set "COURSE_DIR=!BASE_DIR!!FOLDER_NAME!"

echo.
echo ========================================
echo 确认课程信息:
echo ----------------------------------------
echo 课程中文名称: !COURSE_NAME_ZH!
echo 课程英文名称: !COURSE_NAME_EN!
echo 系列中文名称: !SECTION_NAME_ZH!
echo 系列英文名称: !SECTION_NAME_EN!
echo ----------------------------------------
echo 课程ID: !COURSE_ID!
echo 系列ID: !SECTION_ID!
echo 文件夹名称: !FOLDER_NAME!
echo ========================================
echo.

set "CONFIRM=Y"
set /p "CONFIRM=确认创建? [Y]/N: "
if /i not "!CONFIRM!"=="Y" (
    echo 已取消创建
    pause
    exit /b 0
)

if exist "!COURSE_DIR!" (
    echo 错误: 课程文件夹已存在: !COURSE_DIR!
    pause
    exit /b 1
)

echo.
echo 正在创建课程目录结构...
mkdir "!COURSE_DIR!"
mkdir "!COURSE_DIR!\docs"

echo 正在创建 YAML 文件...
call :createYaml

echo 正在创建 index.html...
call :createIndexHtml

echo 正在创建 .nojekyll...
type nul > "!COURSE_DIR!\.nojekyll"

echo 正在创建 docs/README.md...
call :createReadme

echo 正在创建 docs/_sidebar.md...
call :createSidebar

echo.
echo ========================================
echo 课程创建完成!
echo ========================================
echo 课程目录: !COURSE_DIR!
echo.
echo 已创建文件:
echo   - !COURSE_ID!.yaml
echo   - index.html
echo   - .nojekyll
echo   - docs/README.md
echo   - docs/_sidebar.md
echo ========================================
echo.
pause
exit /b 0

:createYaml
set "YAML_FILE=!COURSE_DIR!\!COURSE_ID!.yaml"
> "!YAML_FILE!" echo id: !COURSE_ID!
>> "!YAML_FILE!" echo section: !SECTION_ID!
>> "!YAML_FILE!" echo available: true
>> "!YAML_FILE!" echo link: https://excursion-studio.github.io/ES-courses/!COURSE_ID!/
>> "!YAML_FILE!" echo.
>> "!YAML_FILE!" echo en:
>> "!YAML_FILE!" echo   sectionTitle: !SECTION_NAME_EN!
>> "!YAML_FILE!" echo   title: !COURSE_NAME_EN!
>> "!YAML_FILE!" echo   text: TODO - Add course description in English
>> "!YAML_FILE!" echo   topics:
>> "!YAML_FILE!" echo     - TODO - Add topic 1
>> "!YAML_FILE!" echo     - TODO - Add topic 2
>> "!YAML_FILE!" echo   tags:
>> "!YAML_FILE!" echo     - TODO - Add tag 1
>> "!YAML_FILE!" echo.
>> "!YAML_FILE!" echo zh:
>> "!YAML_FILE!" echo   sectionTitle: !SECTION_NAME_ZH!
>> "!YAML_FILE!" echo   title: !COURSE_NAME_ZH!
>> "!YAML_FILE!" echo   text: TODO - 添加课程中文描述
>> "!YAML_FILE!" echo   topics:
>> "!YAML_FILE!" echo     - TODO - 添加主题1
>> "!YAML_FILE!" echo     - TODO - 添加主题2
>> "!YAML_FILE!" echo   tags:
>> "!YAML_FILE!" echo     - TODO - 添加标签1
exit /b

:createIndexHtml
set "HTML_FILE=!COURSE_DIR!\index.html"
> "!HTML_FILE!" echo ^<!DOCTYPE html^>
>> "!HTML_FILE!" echo ^<html lang="zh-CN"^>
>> "!HTML_FILE!" echo ^<head^>
>> "!HTML_FILE!" echo   ^<meta charset="UTF-8"^>
>> "!HTML_FILE!" echo   ^<title^>加载中...^</title^>
>> "!HTML_FILE!" echo   ^<script^>
>> "!HTML_FILE!" echo     fetch('../template.html'^)
>> "!HTML_FILE!" echo       .then(response =^> response.text(^)^)
>> "!HTML_FILE!" echo       .then(template =^> {
>> "!HTML_FILE!" echo         const html = template.replace(/__BASE_PATH__/g, './docs'^);
>> "!HTML_FILE!" echo         document.open(^);
>> "!HTML_FILE!" echo         document.write(html^);
>> "!HTML_FILE!" echo         document.close(^);
>> "!HTML_FILE!" echo       }^)
>> "!HTML_FILE!" echo       .catch(error =^> {
>> "!HTML_FILE!" echo         console.error('Failed to load template:', error^);
>> "!HTML_FILE!" echo         document.body.innerHTML = '^<p style="color:red;text-align:center;margin-top:50px;"^>模板加载失败，请刷新页面重试^</p^';
>> "!HTML_FILE!" echo       }^);
>> "!HTML_FILE!" echo   ^</script^>
>> "!HTML_FILE!" echo ^</head^>
>> "!HTML_FILE!" echo ^<body^>
>> "!HTML_FILE!" echo   ^<p style="text-align:center;margin-top:50px;color:#667eea;"^>正在加载...^</p^>
>> "!HTML_FILE!" echo ^</body^>
>> "!HTML_FILE!" echo ^</html^>
exit /b

:createReadme
set "README_FILE=!COURSE_DIR!\docs\README.md"
> "!README_FILE!" echo # !COURSE_NAME_ZH!
>> "!README_FILE!" echo.
>> "!README_FILE!" echo ## 课程目标
>> "!README_FILE!" echo.
>> "!README_FILE!" echo TODO - 添加课程目标
>> "!README_FILE!" echo.
>> "!README_FILE!" echo ## 课程内容
>> "!README_FILE!" echo.
>> "!README_FILE!" echo TODO - 添加课程内容
>> "!README_FILE!" echo.
>> "!README_FILE!" echo ## 联系我们
>> "!README_FILE!" echo.
>> "!README_FILE!" echo 如果您在学习过程中遇到任何问题，或者对教程的改进有任何建议，欢迎联系我们：
>> "!README_FILE!" echo.
>> "!README_FILE!" echo - Official Repository: [https://excursion-studio.github.io/ES-courses/](https://github.com/Excursion-Studio/ES-courses)
>> "!README_FILE!" echo - Email: [excursion-studio@outlook.com](mailto:excursion-studio@outlook.com)
exit /b

:createSidebar
set "SIDEBAR_FILE=!COURSE_DIR!\docs\_sidebar.md"
> "!SIDEBAR_FILE!" echo - 目录
>> "!SIDEBAR_FILE!" echo - [课程大纲](README.md)
exit /b

:toLower
set "LOWER_STR=!%1!"
set "LOWER_STR=!LOWER_STR:A=a!"
set "LOWER_STR=!LOWER_STR:B=b!"
set "LOWER_STR=!LOWER_STR:C=c!"
set "LOWER_STR=!LOWER_STR:D=d!"
set "LOWER_STR=!LOWER_STR:E=e!"
set "LOWER_STR=!LOWER_STR:F=f!"
set "LOWER_STR=!LOWER_STR:G=g!"
set "LOWER_STR=!LOWER_STR:H=h!"
set "LOWER_STR=!LOWER_STR:I=i!"
set "LOWER_STR=!LOWER_STR:J=j!"
set "LOWER_STR=!LOWER_STR:K=k!"
set "LOWER_STR=!LOWER_STR:L=l!"
set "LOWER_STR=!LOWER_STR:M=m!"
set "LOWER_STR=!LOWER_STR:N=n!"
set "LOWER_STR=!LOWER_STR:O=o!"
set "LOWER_STR=!LOWER_STR:P=p!"
set "LOWER_STR=!LOWER_STR:Q=q!"
set "LOWER_STR=!LOWER_STR:R=r!"
set "LOWER_STR=!LOWER_STR:S=s!"
set "LOWER_STR=!LOWER_STR:T=t!"
set "LOWER_STR=!LOWER_STR:U=u!"
set "LOWER_STR=!LOWER_STR:V=v!"
set "LOWER_STR=!LOWER_STR:W=w!"
set "LOWER_STR=!LOWER_STR:X=x!"
set "LOWER_STR=!LOWER_STR:Y=y!"
set "LOWER_STR=!LOWER_STR:Z=z!"
set "%1=!LOWER_STR!"
exit /b
