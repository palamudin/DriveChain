# side-by-side old-raw vs new-showcase comparison: python3 tools/compare.py a.png b.png out.png
import sys
from PIL import Image, ImageDraw, ImageFont
a, b, out = sys.argv[1:4]
A, B = Image.open(a).convert('RGB'), Image.open(b).convert('RGB')
w, h = A.size; W = w // 2; H = h // 2
A, B = A.resize((W, H), Image.LANCZOS), B.resize((W, H), Image.LANCZOS)
C = Image.new('RGB', (W * 2 + 8, H + 40), (12, 12, 14)); C.paste(A, (0, 40)); C.paste(B, (W + 8, 40))
d = ImageDraw.Draw(C)
try: f = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 22)
except Exception: f = ImageFont.load_default()
d.text((12, 8), 'OLD: raw look', fill=(230, 230, 230), font=f); d.text((W + 20, 8), 'NEW: showcase tier', fill=(230, 230, 230), font=f)
C.save(out); print(out, C.size)
