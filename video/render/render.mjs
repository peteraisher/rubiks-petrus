// Renders the video frame by frame in headless Chromium and pipes the
// frames to ffmpeg.
//   node render.mjs                 -> out/petrus-race.mp4
//   node render.mjs --still 3.5 9   -> out/still-3.5.png, out/still-9.png
import { createServer } from 'node:http';
import { readFile, mkdir } from 'node:fs/promises';
import { spawn } from 'node:child_process';
import { extname, join, normalize } from 'node:path';
import { homedir } from 'node:os';
import { chromium } from 'playwright-core';

const ROOT = new URL('.', import.meta.url).pathname;
const OUT = join(ROOT, 'out');
const WIDTH = 1080;
const HEIGHT = 1350;
const FPS = 60;
const CRF = '16';
const TYPES = { '.html': 'text/html', '.js': 'text/javascript', '.json': 'application/json' };
const CHROME = process.env.CHROME ||
  join(homedir(), 'Library/Caches/ms-playwright/chromium-1234/chrome-mac-arm64/Google Chrome for Testing.app/Contents/MacOS/Google Chrome for Testing');

const server = createServer(async (req, res) => {
  const path = normalize(join(ROOT, decodeURIComponent(new URL(req.url, 'http://x').pathname)));
  if (!path.startsWith(ROOT)) return res.writeHead(403).end();
  try {
    const body = await readFile(path.endsWith('/') ? join(path, 'index.html') : path);
    res.writeHead(200, { 'content-type': TYPES[extname(path)] || 'application/octet-stream' }).end(body);
  } catch {
    res.writeHead(404).end();
  }
}).listen(0, '127.0.0.1');
await new Promise((r) => server.once('listening', r));

const browser = await chromium.launch({ executablePath: CHROME, args: ['--use-angle=metal', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: WIDTH, height: HEIGHT }, deviceScaleFactor: 1 });
page.on('pageerror', (e) => console.error('page error:', e.message));
await page.goto(`http://127.0.0.1:${server.address().port}/index.html`);
await page.waitForFunction(() => window.ready || window.error, null, { timeout: 60000 });
const err = await page.evaluate(() => window.error);
if (err) throw new Error(err);
await mkdir(OUT, { recursive: true });

const args = process.argv.slice(2);
if (args[0] === '--still') {
  for (const s of args.slice(1)) {
    await page.evaluate((i) => window.renderFrame(i), Math.round(Number(s) * FPS));
    await page.screenshot({ path: join(OUT, `still-${s}.png`) });
    console.log(`out/still-${s}.png`);
  }
} else {
  const frames = await page.evaluate(() => window.frameCount);
  const file = join(OUT, 'petrus-race.mp4');
  const ff = spawn('ffmpeg', ['-y', '-loglevel', 'error', '-f', 'image2pipe', '-framerate', String(FPS), '-i', '-',
    '-c:v', 'libx264', '-preset', 'slow', '-crf', CRF, '-pix_fmt', 'yuv420p', '-movflags', '+faststart', file],
  { stdio: ['pipe', 'inherit', 'inherit'] });
  const t0 = Date.now();
  for (let i = 0; i < frames; i++) {
    await page.evaluate((i) => window.renderFrame(i), i);
    const png = await page.screenshot({ type: 'png' });
    if (!ff.stdin.write(png)) await new Promise((r) => ff.stdin.once('drain', r));
    if (i % FPS === 0) process.stdout.write(`\r${i}/${frames} frames, ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  }
  ff.stdin.end();
  await new Promise((r, j) => ff.on('close', (c) => (c ? j(new Error(`ffmpeg exit ${c}`)) : r())));
  console.log(`\n${file}`);
}
await browser.close();
server.close();
