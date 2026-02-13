## 目标
将 convex-optimization、differential-geometry、stochastic-process 三个目录中完全相同的 index.html 代码提取到根目录的 template.html 中，然后简化各课程的 index.html 为动态加载模板的形式。

## 具体步骤

### 1. 创建 template.html 模板文件
- 将现有的完整 index.html 内容写入 template.html
- 将 `basePath: './docs'` 改为可配置的占位符 `__BASE_PATH__`

### 2. 简化各课程的 index.html
三个课程的 index.html 将简化为以下结构：
- 使用 fetch 加载 ../template.html
- 替换 `__BASE_PATH__` 为 `'./docs'`
- 将处理后的 HTML 写入 document

### 3. 验证
- 确保三个课程的 index.html 都能正常工作
- 确认 Docsify 配置正确加载

## 文件变更
- **新增/修改**: `template.html` - 完整的模板文件
- **修改**: `convex-optimization/index.html` - 简化为加载器
- **修改**: `differential-geometry/index.html` - 简化为加载器
- **修改**: `stochastic-process/index.html` - 简化为加载器