"""Render the real local console in an isolated headless Chrome for visual QA."""
import argparse
import html
import json
from pathlib import Path
import re
import subprocess
import sys
import threading
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'App'))
from desktop_server import ConsoleService,ConsoleHTTPServer

p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--width',type=int,default=1600);p.add_argument('--height',type=int,default=1100);p.add_argument('--internal',action='store_true');p.add_argument('--view',choices=('tracking','video'),default='tracking');p.add_argument('--mode',type=int,choices=range(6),default=0);args=p.parse_args()
args.output=args.output.resolve()
args.output.mkdir(parents=True,exist_ok=True)
service=ConsoleService(preview=True,demo=True);server=ConsoleHTTPServer(service)
service.state['video_mode']=args.mode  # Preview-only confirmed-mode fixture, no serial connection.
thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
try:
    browser=Path('C:/Program Files/Google/Chrome/Application/chrome.exe')
    suffix='_internal' if args.internal else '_video' if args.view=='video' else ''
    screenshot=args.output/f'console_{args.width}{suffix}.png'
    commands=[str(browser),'--headless=new','--no-first-run','--no-default-browser-check','--disable-background-networking',
              '--user-data-dir='+str(args.output/f'chrome_profile_{args.width}'),'--hide-scrollbars',
              f'--window-size={args.width},{args.height}','--force-device-scale-factor=1',
              '--screenshot='+str(screenshot),'--dump-dom','--virtual-time-budget=5000',server.url+f'?qa=1&view={args.view}&internal={int(args.internal)}']
    result=subprocess.run(commands,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=55)
    dom=result.stdout.decode('utf-8','replace')
    (args.output/f'console_{args.width}{suffix}.html').write_text(dom,encoding='utf-8')
    match=re.search(r'<pre id="qa-result" hidden="">(.*?)</pre>',dom,re.S)
    report=json.loads(html.unescape(match[1])) if match else dict(passed=False,error='Browser QA result missing',stderr=result.stderr.decode('utf-8','replace')[-1800:])
    report.update(width=args.width,height=args.height,screenshot=screenshot.name,exit_code=result.returncode)
    (args.output/f'browser_check_{args.width}{suffix}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=True))
    if not report['passed']:raise SystemExit(1)
finally:
    service.close();server.shutdown();server.server_close()
