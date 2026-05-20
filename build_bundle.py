"""Build a single-file dips.html by inlining CSS + JS into index.html."""
import os, re

ROOT = os.path.join(os.path.dirname(__file__), "docs")

with open(os.path.join(ROOT, "index.html"), encoding="utf-8") as f:
    html = f.read()

# Inline CSS
with open(os.path.join(ROOT, "css", "style.css"), encoding="utf-8") as f:
    css = f.read()
html = html.replace(
    '<link rel="stylesheet" href="css/style.css" />',
    f"<style>\n{css}\n</style>",
)

# Drop laz-perf CDN (LAZ decompression not supported in browser bundle)
html = html.replace(
    '<script src="https://cdn.jsdelivr.net/npm/laz-perf@0.0.7/lib/web/laz-perf.js"></script>\n',
    "",
)

# Inline local JS files
for js_name in ("faa_core.js", "las_loader.js", "app.js"):
    with open(os.path.join(ROOT, "js", js_name), encoding="utf-8") as f:
        js = f.read()
    html = html.replace(
        f'<script src="js/{js_name}"></script>',
        f"<script>\n{js}\n</script>",
    )

out = os.path.join(os.path.dirname(__file__), "dips.html")
with open(out, "w", encoding="utf-8") as f:
    f.write(html)

print(f"Wrote {out} ({os.path.getsize(out)//1024} KB)")
