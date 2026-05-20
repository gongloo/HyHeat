import os
import gzip
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@gongloo"]
DEPENDENCIES = ["web_server_base"]
MULTI_CONF = True

html_host_ns = cg.esphome_ns.namespace("html_host")
HtmlHostComponent = html_host_ns.class_("HtmlHostComponent", cg.Component)

CONF_HTML_FILE = "html_file"
CONF_URL = "url"
CONF_CONTENT_TYPE = "content_type"

def detect_content_type(filepath):
    ext = os.path.splitext(filepath)[1].lower()
    if ext == ".html" or ext == ".htm":
        return "text/html"
    elif ext == ".css":
        return "text/css"
    elif ext == ".js":
        return "application/javascript"
    elif ext == ".json" or ext == ".webmanifest":
        return "application/json"
    elif ext == ".png":
        return "image/png"
    elif ext == ".ico":
        return "image/x-icon"
    elif ext == ".svg":
        return "image/svg+xml"
    elif ext == ".txt":
        return "text/plain"
    else:
        return "application/octet-stream"

def validate_html_file(value):
    from esphome.core import CORE
    path = cv.string(value)
    if not os.path.isabs(path):
        path = os.path.join(CORE.config_dir, path)
    if not os.path.exists(path):
        raise cv.Invalid(f"HTML/resource file not found at path: {path}")
    return value

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(HtmlHostComponent),
    cv.Required(CONF_HTML_FILE): validate_html_file,
    cv.Required(CONF_URL): cv.string,
    cv.Optional(CONF_CONTENT_TYPE): cv.string,
}).extend(cv.COMPONENT_SCHEMA)

def get_mdi_path(icon_name):
    # Dynamic compile-time MDI SVG path downloader.
    # Natively calls ESPHome's internal download_gh_svg utility to pull, cache, and resolve
    # icons inside the .esphome workspace, eliminating code duplication!
    from esphome.components.image import download_gh_svg, SOURCE_MDI
    import re
    
    icon_name = "".join(c for c in icon_name.lower() if c.isalnum() or c in "-_")
    
    try:
        # Call ESPHome's native downloader/cacher!
        # This automatically resolves the path, downloads it if missing, and returns the absolute local path.
        svg_file_path = download_gh_svg(icon_name, SOURCE_MDI)
        
        with open(svg_file_path, "r", encoding="utf-8") as f:
            svg_content = f.read()
            match = re.search(r'<path\s+d="([^"]+)"', svg_content)
            if match:
                return match.group(1)
    except Exception as e:
        raise RuntimeError(
            f"Failed to fetch or parse MDI icon '{icon_name}' natively. "
            "Please ensure you have an active internet connection during initial compilation."
        ) from e

    raise RuntimeError(f"Icon path not found in fetched SVG for '{icon_name}'")

async def to_code(config):
    from esphome.core import CORE
    
    html_file = config[CONF_HTML_FILE]
    url = config[CONF_URL]
    
    if not os.path.isabs(html_file):
        html_file = os.path.join(CORE.config_dir, html_file)
        
    if CONF_CONTENT_TYPE in config:
        content_type = config[CONF_CONTENT_TYPE]
    else:
        content_type = detect_content_type(html_file)
        
    # Read resource as bytes
    with open(html_file, "rb") as f:
        file_content = f.read()

    # Preprocess all <svg ... data-mdi="icon_name">...</svg> elements for HTML files
    if content_type == "text/html":
        try:
            html_text = file_content.decode("utf-8")
            import re
            pattern = r'(<svg\b[^>]*data-mdi="([^"]+)"[^>]*>)(.*?)(</svg>)'
            
            def mdi_replacer(match):
                opening_tag = match.group(1)
                icon_name = match.group(2)
                try:
                    path_data = get_mdi_path(icon_name)
                    return f'{opening_tag}<path d="{path_data}"/></svg>'
                except Exception as e:
                    return match.group(0)
                    
            html_text = re.sub(pattern, mdi_replacer, html_text, flags=re.DOTALL)
            file_content = html_text.encode("utf-8")
        except UnicodeDecodeError:
            pass

    gzipped_bytes = gzip.compress(file_content)
    hex_bytes_str = ", ".join(f"0x{b:02x}" for b in gzipped_bytes)

    # Generate a unique variable name based on the C++ variable ID to avoid collisions
    var_id = str(config[CONF_ID])
    
    header_content = f"""#pragma once
#include "esphome/core/progmem.h"
#include <stddef.h>

// Automatically generated during ESPHome compile phase. Do not edit.
constexpr uint8_t {var_id}_HTML_GZ[] PROGMEM = {{{hex_bytes_str}}};
constexpr size_t {var_id}_HTML_GZ_SIZE = {len(gzipped_bytes)};
"""

    build_dir = os.path.join(CORE.build_path, "src")
    os.makedirs(build_dir, exist_ok=True)
    header_filename = f"{var_id}_html.h"
    output_header_path = os.path.join(build_dir, header_filename)
    
    with open(output_header_path, "w", encoding="utf-8") as f:
        f.write(header_content)
        
    # Include header dynamically in ESPHome build
    cg.add_global(cg.RawStatement(f'#include "{header_filename}"'))
    
    var = cg.new_Pvariable(
        config[CONF_ID],
        url,
        content_type,
        cg.RawExpression(f"{var_id}_HTML_GZ"),
        cg.RawExpression(f"{var_id}_HTML_GZ_SIZE")
    )
    await cg.register_component(var, config)
