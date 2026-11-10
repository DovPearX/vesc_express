const shapeNames = ['rect', 'circle', 'line', 'rect-outline', 'circle-outline', 'ellipse', 'arc', 'pie', 'triangle', 'round-rect', 'round-rect-outline', 'text', 'image'];
const fillNames = ['solid', 'gradient-x', 'gradient-y', 'gradient'];
const editor = document.querySelector('#playground');
const outputCode = document.querySelector('#live-code');
const canvas = document.querySelector('#live-preview');
const context = canvas.getContext('2d');
let renderer, assetImage, assetName = 'logo.bin', fontAssetName = 'font.bin', fontBytes;
let current = normalizeParameters(examples[0].parameters), currentText = examples[0].text;
let scheduled = false;

function normalizeParameters(values) {
  return Array.from({length: 56}, (_, i) => values[i] ?? 0);
}
const ranges = [
  [20, 'X', -160, 320, 1, 'geometry'], [21, 'Y', -100, 180, 1, 'geometry'],
  [22, 'Width / radius / X2', 1, 320, 1, 'geometry'], [23, 'Height / radius Y / Y2', 1, 180, 1, 'geometry'],
  [24, 'Arc start', -180, 180, 1, 'arc'], [25, 'Arc sweep', -360, 360, 1, 'arc'],
  [29, 'Gradient angle', -180, 180, 1, 'gradient'],
  [26, 'Corner radius', 0, 80, 1, 'rounded'], [27, 'Triangle X3', -160, 320, 1, 'triangle'],
  [28, 'Triangle Y3', -100, 180, 1, 'triangle'], [10, 'Stroke width', 1, 16, 1, 'stroke'],
  [11, 'Dash length', 0, 32, 1, 'line'], [12, 'Gap length', 0, 24, 1, 'line'],
  [2, 'Opacity', 0, 255, 1, 'all'], [4, 'Rotation', -180, 180, 1, 'primitive'],
  [5, 'Skew Q8', -256, 256, 8, 'primitive'], [7, 'Wave amplitude', 0, 20, 1, 'primitive'],
  [30, 'Wave period', 8, 160, 1, 'wave'], [31, 'Wave phase', 0, 360, 1, 'wave'],
  [55, 'Imported font size (px)', 1, 128, 1, 'imported-font'],
  [6, 'Zoom Q8 (256 = 1x)', 64, 512, 16, 'primitive'], [14, 'Font scale', 1, 8, 1, 'text']
];
document.querySelector('#live-kind').innerHTML = [
  ...shapeNames.map((name, i) => `<option value="${i}">${name}</option>`)
].join('');
document.querySelector('#live-ranges').innerHTML = ranges.map(([index, label, min, max, step, group]) =>
  `<label class="range-control" data-group-control="${group}"><span>${label}<output id="value-${index}"></output></span><input type="range" data-parameter="${index}" min="${min}" max="${max}" step="${step}" aria-label="${label}"></label>`
).join('');
function hex(value) { return `0x${(value & 65535).toString(16).padStart(4, '0')}`; }
function rgb565(value) {
  const rgb = parseInt(value.slice(1), 16);
  return ((rgb >>> 19) << 11) | (((rgb >>> 10) & 63) << 5) | ((rgb >>> 3) & 31);
}
function htmlColor(value) {
  const r = value >>> 11, g = (value >>> 5) & 63, b = value & 31;
  return `#${(((r << 3 | r >>> 2) << 16) | ((g << 2 | g >>> 4) << 8) | (b << 3 | b >>> 2)).toString(16).padStart(6, '0')}`;
}
function groupVisible(group) {
  const kind = current[0];
  switch (group) {
    case 'primitive': return kind < 12;
    case 'wave': return kind < 12 && current[7] > 0;
    case 'font-asset': return kind === 11;
    case 'imported-font': return Boolean(current[54]) && groupVisible('font-asset');
    case 'gradient': return current[1] === 3;
    case 'arc': return kind === 6 || kind === 7;
    case 'rounded': return kind === 9 || kind === 10;
    case 'triangle': return kind === 8;
    case 'stroke': return [2, 3, 4, 6, 10].includes(kind);
    case 'line': return kind === 2;
    case 'text': return !current[54] && groupVisible('font-asset');
    case 'image': return kind === 12;
    default: return true;
  }
}
function syncControls() {
  const kind = current[0];
  document.querySelector('#live-kind').value = kind;
  const fill = document.querySelector('#live-fill');
  const fillAllowed = value => renderer.tinysg_preview_fill_allowed(kind, value);
  fill.innerHTML = fillNames.flatMap((name, i) => fillAllowed(i) ? [`<option value="${i}">${name}</option>`] : []).join('');
  if (!fillAllowed(current[1])) current[1] = 0;
  fill.value = current[1]; fill.closest('label').hidden = kind === 12;
  document.querySelector('#live-secondary').closest('label').hidden = kind < 100 && !current[1];
  for (const control of editor.querySelectorAll('[data-parameter]')) {
    const index = Number(control.dataset.parameter);
    if (control.type === 'checkbox') control.checked = Boolean(current[index]);
    else control.value = current[index];
    const value = document.querySelector(`#value-${index}`); if (value) value.textContent = current[index];
  }
  for (const control of editor.querySelectorAll('[data-group-control]')) control.hidden = !groupVisible(control.dataset.groupControl);
  for (const index of [22, 23]) {
    const control = editor.querySelector(`[data-parameter="${index}"]`);
    control.closest('label').hidden = kind === 11 || (index === 23 && [1, 4, 6, 7].includes(kind));
    const label = kind === 2 || kind === 8 ? (index === 22 ? 'X2' : 'Y2') : [1, 4, 5, 6, 7].includes(kind) ? (index === 22 ? 'Radius X' : 'Radius Y') : index === 22 ? 'Width' : 'Height';
    control.closest('label').querySelector('span').firstChild.textContent = label;
  }
  document.querySelector('#live-quality').value = current[3];
  document.querySelector('#live-quality').closest('label').hidden = kind === 12;
  document.querySelector('#live-font').value = current[13];
  document.querySelector('#live-font').closest('label').hidden = kind !== 11 || Boolean(current[54]);
  document.querySelector('#live-text').value = currentText;
  document.querySelector('#live-text').closest('label').hidden = !groupVisible('font-asset');
  document.querySelector('#live-use-font').checked = Boolean(current[54]);
  document.querySelector('#live-text').previousSibling.textContent = 'ASCII text';
  for (const [id, index] of [['live-primary', 15], ['live-secondary', 16]]) document.querySelector(`#${id}`).value = htmlColor(current[index]);
  document.querySelector('#live-tip').textContent = kind === 12
    ? 'Export uses the tinyGFX RGB565 BIN format. Download the asset and keep it beside your Lisp program.'
    : 'Solid colors and gradients. FAST is the default; AA2 and AA4 increase CPU work.';
}
function fontOption() {
  return current[54] ? `(list 'font ui-font ${current[55]})` : null;
}
function makeCode() {
  const code = constructorCode();
  return current[54] && groupVisible('font-asset') ? `(import ${JSON.stringify(fontAssetName)} 'ui-font)\n${code}` : code;
}
function constructorCode() {
  const p = current, kind = p[0];
  if (kind === 12) return `(import ${JSON.stringify(assetName)} 'image-data)\n(tinysg 'render 0x0862 (list\n  (tinysg 'image '(${p.slice(20, 22).join(' ')}) image-data${p[2] !== 255 ? ` '(opacity ${p[2]})` : ''})))`;
  let pivotX = Math.trunc(p[22] / 2), pivotY = Math.trunc(p[23] / 2);
  if (kind === 2) { pivotX = Math.trunc((p[22] - p[20]) / 2); pivotY = Math.trunc((p[23] - p[21]) / 2); }
  if ([1, 4, 5, 6, 7, 8].includes(kind)) pivotX = pivotY = 0;
  if (kind === 11 && !p[54]) { pivotX = Math.trunc(Math.max(0, currentText.length * (p[13] + 1) - 1) * p[14] / 2); pivotY = Math.trunc((p[13] === 3 ? 5 : 7) * p[14] / 2); }
  if (kind === 11 && p[54] && fontBytes) {
    let width = 0;
    for (const letter of currentText) {
      const code = letter.charCodeAt(0), index = code - fontBytes[4];
      width += index >= 0 && index < fontBytes[5] ? fontBytes[8 + index * 8 + 5] : 0;
    }
    pivotX = Math.floor(Math.ceil(width * p[55] / fontBytes[7]) / 2);
    pivotY = Math.floor(Math.ceil(fontBytes[6] * p[55] / fontBytes[7]) / 2);
  }
  if ([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11].includes(kind)) {
    const options = [];
    if ([0, 1, 9].includes(kind)) options.push("'(filled)");
    if (kind === 9 || kind === 10) options.push(`'(rounded ${p[26]})`);
    if ([2, 3, 4, 6, 10].includes(kind) && p[10] !== 1) options.push(`'(thickness ${p[10]})`);
    if (kind === 2 && p[11]) options.push(`'(dashed ${p[11]} ${p[12]})`);
    if (kind === 11) options.push(fontOption() || `'(font ${p[13]} ${p[14]})`);
    if (p[1]) options.push(`'(fill ${fillNames[p[1]]} ${hex(p[16])}${p[1] === 3 ? ` ${p[29]}` : ''})`);
    if (p[2] !== 255) options.push(`'(opacity ${p[2]})`);
    if (p[3]) options.push(`'(quality ${p[3] === 1 ? 'aa2' : 'aa4'})`);
    if (p[4]) options.push(`'(rotate ${p[4]})`);
    if (p[5]) options.push(`'(skew ${p[5]})`);
    if (p[7]) options.push(`'(wave ${p[7]} ${p[30]} ${p[31]})`);
    if (p[6] !== 256) options.push(`'(zoom ${p[6]} ${p[6]})`);
    if (p[4] || p[5] || p[6] !== 256 || p[7]) options.push(`'(pivot ${pivotX} ${pivotY})`);
    const name = [0, 3, 9, 10].includes(kind) ? 'rect' : kind === 1 || kind === 4 ? 'circle' : kind === 2 ? 'line' : kind === 6 ? 'arc' : kind === 7 ? 'pie' : kind === 5 ? 'ellipse' : kind === 8 ? 'triangle' : 'text';
    const geometry = kind === 8 ? `${p.slice(20, 24).join(' ')} ${p[27]} ${p[28]} ${hex(p[15])}` : kind === 6 || kind === 7 ? `${p[20]} ${p[21]} ${p[22]} ${hex(p[15])} ${p[24]} ${p[25]}` : kind === 11 ? `${p[20]} ${p[21]} ${hex(p[15])} ${JSON.stringify(currentText)}` : `${p.slice(20, kind === 1 || kind === 4 ? 23 : 24).join(' ')} ${hex(p[15])}`;
    return `(tinysg 'render 0x0862 (list\n  (tinysg '${name} ${geometry}${options.length ? '\n    ' + options.join(' ') : ''})))`;
  }

}
function imageBytes() {
  if (!assetImage) return null;
  const width = Math.max(1, current[22]), height = Math.max(1, current[23]);
  const imageCanvas = document.createElement('canvas'); imageCanvas.width = width; imageCanvas.height = height;
  const ctx = imageCanvas.getContext('2d'); ctx.fillStyle = '#080c10'; ctx.fillRect(0, 0, width, height); ctx.drawImage(assetImage, 0, 0, width, height);
  const rgba = ctx.getImageData(0, 0, width, height).data, bytes = new Uint8Array(width * height * 2);
  for (let i = 0; i < width * height; i++) {
    const color = ((rgba[i * 4] >>> 3) << 11) | ((rgba[i * 4 + 1] >>> 2) << 5) | (rgba[i * 4 + 2] >>> 3);
    bytes[i * 2] = color >>> 8; bytes[i * 2 + 1] = color & 255;
  }
  return bytes;
}
function render() {
  if (!renderer) return; scheduled = false;
  new Int32Array(renderer.memory.buffer, renderer.tinysg_preview_parameters(), 56).set(current);
  const bytes = new Uint8Array(renderer.memory.buffer, renderer.tinysg_preview_text(), 128); bytes.fill(0); bytes.set(new TextEncoder().encode(currentText).slice(0, 127));
  if (current[0] === 12) { const pixels = imageBytes(); if (pixels) new Uint8Array(renderer.memory.buffer, renderer.tinysg_preview_image(), pixels.length).set(pixels); }
  outputCode.textContent = makeCode();
  const start = performance.now(), pixels = renderer.tinysg_preview();
  if (!pixels) { document.querySelector('#live-runtime').textContent = 'Invalid dimensions, items or style.'; return; }
  context.putImageData(new ImageData(new Uint8ClampedArray(renderer.memory.buffer, pixels, 320 * 180 * 4), 320, 180), 0, 0);
  document.querySelector('#live-runtime').textContent = `C / WebAssembly / RGB565 / ${(performance.now() - start).toFixed(1)} ms`;
}
function changed() { if (!renderer) return; syncControls(); if (!scheduled) { scheduled = true; requestAnimationFrame(render); } }
function choose(example) { current = normalizeParameters(example.parameters); currentText = example.text; changed(); }
editor.addEventListener('input', event => {
  const target = event.target;
  if (target.dataset.parameter) current[Number(target.dataset.parameter)] = Number(target.type === 'checkbox' ? target.checked : target.value);
  for (const [id, index] of [['live-fill', 1], ['live-quality', 3], ['live-font', 13]]) if (target.id === id) current[index] = Number(target.value);
  for (const [id, index] of [['live-primary', 15], ['live-secondary', 16]]) if (target.id === id) current[index] = rgb565(target.value);
  if (target.id === 'live-text') currentText = target.value.replace(/[^\x20-\x7e]/g, '?');
  if (target.id === 'live-kind') { choose(examples.find(e => e.parameters[0] === Number(target.value))); return; }
  changed();
});
document.querySelector('#live-reset').addEventListener('click', () => choose(examples.find(e => e.parameters[0] === current[0])));
document.querySelector('#live-copy').addEventListener('click', async event => {
  try { await navigator.clipboard.writeText(outputCode.textContent); event.target.textContent = 'Copied'; }
  catch { const range = document.createRange(); range.selectNodeContents(outputCode); const selection = getSelection(); selection.removeAllRanges(); selection.addRange(range); event.target.textContent = 'Select / copy code'; }
});
gallery.addEventListener('click', event => { const picture = event.target.closest('.picture'); if (!picture) return; choose(examples[Number(picture.closest('.card').id.slice(8))]); editor.scrollIntoView({behavior: 'smooth', block: 'start'}); });
document.querySelector('#live-font-asset').addEventListener('change', async event => {
  const file = event.target.files[0]; if (!file) return;
  const bytes = new Uint8Array(await file.arrayBuffer());
  if (bytes.length > 262144) { document.querySelector('#live-font-status').textContent = 'Font limit: 256 KiB.'; return; }
  new Uint8Array(renderer.memory.buffer, renderer.tinysg_preview_font(), bytes.length).set(bytes);
  if (!renderer.tinysg_preview_load_font(bytes.length)) {
    current[54] = 0; fontBytes = null; changed();
    document.querySelector('#live-use-font').disabled = true;
    document.querySelector('#live-font-status').textContent = 'Expected a TSF1 font from tools/font.py.';
    return;
  }
  fontBytes = bytes; fontAssetName = file.name; current[54] = 1;
  document.querySelector('#live-use-font').disabled = false;
  document.querySelector('#live-font-status').textContent = file.name + ' · ' + Math.ceil(bytes.length / 1024) + ' KiB';
  changed();
});

document.querySelector('#live-asset').addEventListener('change', async event => {
  const file = event.target.files[0]; if (!file) return;
  try { assetImage = await createImageBitmap(file); assetName = file.name.replace(/\.[^.]+$/, '') + '.bin'; changed(); }
  catch { document.querySelector('#live-runtime').textContent = 'Cannot decode this image.'; }
});
document.querySelector('#live-download-image').addEventListener('click', () => {
  const bytes = imageBytes(); if (!bytes) return;
  const header = new Uint8Array([current[22] >>> 8, current[22] & 255, current[23] >>> 8, current[23] & 255, 16]);
  const url = URL.createObjectURL(new Blob([header, bytes], {type: 'application/octet-stream'}));
  const link = document.createElement('a'); link.href = url; link.download = assetName; link.click(); URL.revokeObjectURL(url);
});
async function initialize() {
  const saved = current, savedText = currentText;
  for (const example of examples) { current = normalizeParameters(example.parameters); currentText = example.text; example.code = makeCode(); }
  current = saved; currentText = savedText; draw();
  try {
    const bytes = Uint8Array.from(atob(document.querySelector('#renderer-wasm').textContent.trim()), c => c.charCodeAt(0));
    renderer = (await WebAssembly.instantiate(bytes)).instance.exports;
    assetImage = new Image(); assetImage.src = 'data:image/png;base64,' + document.querySelector('#sample-image').textContent.trim(); await assetImage.decode();
    editor.querySelector('fieldset').disabled = false; choose(examples[0]);
  } catch (error) { document.querySelector('#live-runtime').textContent = error.message; }
}
initialize();
