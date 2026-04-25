(function () {
  'use strict';

  function isHomePage() {
    var path = window.location.pathname;
    return path === '/' || path === '/index.html';
  }

  // Fallback poem pool — used when API is unreachable
  var fallbackPoems = [
    { line: '等闲识得东风面，万紫千红总是春。', source: '朱熹《春日》' },
    { line: '落霞与孤鹜齐飞，秋水共长天一色。', source: '王勃《滕王阁序》' },
    { line: '接天莲叶无穷碧，映日荷花别样红。', source: '杨万里《晓出净慈寺送林子方》' },
    { line: '晴空一鹤排云上，便引诗情到碧霄。', source: '刘禹锡《秋词》' },
    { line: '忽如一夜春风来，千树万树梨花开。', source: '岑参《白雪歌送武判官归京》' },
    { line: '千山鸟飞绝，万径人踪灭。', source: '柳宗元《江雪》' },
    { line: '停车坐爱枫林晚，霜叶红于二月花。', source: '杜牧《山行》' },
    { line: '但愿人长久，千里共婵娟。', source: '苏轼《水调歌头》' },
    { line: '会当凌绝顶，一览众山小。', source: '杜甫《望岳》' },
    { line: '举头望明月，低头思故乡。', source: '李白《静夜思》' },
    { line: '春蚕到死丝方尽，蜡炬成灰泪始干。', source: '李商隐《无题》' },
    { line: '采菊东篱下，悠然见南山。', source: '陶渊明《饮酒》' },
    { line: '人生自古谁无死，留取丹心照汗青。', source: '文天祥《过零丁洋》' },
    { line: '海内存知己，天涯若比邻。', source: '王勃《送杜少府之任蜀州》' },
    { line: '欲穷千里目，更上一层楼。', source: '王之涣《登鹳雀楼》' },
    { line: '烽火连三月，家书抵万金。', source: '杜甫《春望》' },
    { line: '大漠孤烟直，长河落日圆。', source: '王维《使至塞上》' },
    { line: '黄河之水天上来，奔流到海不复回。', source: '李白《将进酒》' },
    { line: '问渠那得清如许，为有源头活水来。', source: '朱熹《观书有感》' },
    { line: '不识庐山真面目，只缘身在此山中。', source: '苏轼《题西林壁》' },
    { line: '春风得意马蹄疾，一日看尽长安花。', source: '孟郊《登科后》' },
    { line: '疏影横斜水清浅，暗香浮动月黄昏。', source: '林逋《山园小梅》' },
    { line: '醉卧沙场君莫笑，古来征战几人回。', source: '王翰《凉州词》' },
    { line: '桃花潭水深千尺，不及汪伦送我情。', source: '李白《赠汪伦》' },
    { line: '少壮不努力，老大徒伤悲。', source: '《长歌行》' },
    { line: '春色满园关不住，一枝红杏出墙来。', source: '叶绍翁《游园不值》' },
    { line: '沉舟侧畔千帆过，病树前头万木春。', source: '刘禹锡《酬乐天扬州初逢席上见赠》' },
    { line: '山重水复疑无路，柳暗花明又一村。', source: '陆游《游山西村》' },
    { line: '天生我材必有用，千金散尽还复来。', source: '李白《将进酒》' },
    { line: '安能摧眉折腰事权贵，使我不得开心颜。', source: '李白《梦游天姥吟留别》' },
    { line: '横眉冷对千夫指，俯首甘为孺子牛。', source: '鲁迅《自嘲》' },
    { line: '宝剑锋从磨砺出，梅花香自苦寒来。', source: '《警世贤文》' }
  ];

  // Rotating index: random start, increments on each fallback call
  var fallbackIndex = Math.floor(Math.random() * fallbackPoems.length);

  // ── jinrishici v2 token management ────────────────────────────────────────
  // ROOT CAUSE OF ALWAYS-SAME-POEM BUG:
  // Without X-User-Token header the API treats every call as a brand-new
  // anonymous session and always returns poem #0 (春江花月夜).
  // FIX: persist the token the server returns, send it on the next request.
  var TOKEN_KEY = 'jrs_v2_token';

  function getStoredToken() {
    try { return localStorage.getItem(TOKEN_KEY) || ''; } catch (e) { return ''; }
  }

  function saveToken(t) {
    try { if (t) localStorage.setItem(TOKEN_KEY, t); } catch (e) {}
  }

  function fetchPoetryFromAPI(callback) {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', 'https://v2.jinrishici.com/one.json', true);
    xhr.timeout = 5000;
    var token = getStoredToken();
    if (token) xhr.setRequestHeader('X-User-Token', token);
    xhr.onreadystatechange = function () {
      if (xhr.readyState !== 4) return;
      // Persist token from response for the next request
      try { saveToken(xhr.getResponseHeader('X-User-Token')); } catch (e) {}
      if (xhr.status === 200) {
        try {
          var resp = JSON.parse(xhr.responseText);
          if (resp.status === 'success' && resp.data && resp.data.content) {
            var origin = resp.data.origin || {};
            var src = (origin.author || '') +
              (origin.title ? '\u300a' + origin.title + '\u300b' : '');
            callback({ line: resp.data.content, source: src });
            return;
          }
        } catch (e) {}
      }
      callback(null);
    };
    xhr.onerror   = function () { callback(null); };
    xhr.ontimeout = function () { callback(null); };
    xhr.send();
  }

  // Fallback: rotate index so each call returns a different poem
  function getFallbackPoem() {
    fallbackIndex = (fallbackIndex + 1) % fallbackPoems.length;
    return fallbackPoems[fallbackIndex];
  }

  // ── DOM ───────────────────────────────────────────────────────────────────
  function buildHero() {
    var siteInfo = document.getElementById('site-info');
    if (!siteInfo || document.querySelector('.jj-verse')) return;

    // Hide default title/subtitle — poem is the visual centrepiece
    ['site-title', 'site-subtitle'].forEach(function (id) {
      var el = document.getElementById(id);
      if (el) el.style.display = 'none';
    });

    var wrap   = document.createElement('div');
    wrap.className = 'jj-verse';

    var deco   = document.createElement('div');
    deco.className = 'jj-verse-deco';
    deco.setAttribute('aria-hidden', 'true');

    var lineEl = document.createElement('div');
    lineEl.className = 'jj-verse-line';
    lineEl.textContent = '\u2026';

    var srcEl  = document.createElement('div');
    srcEl.className = 'jj-verse-src';

    var btn    = document.createElement('button');
    btn.className   = 'jj-verse-btn';
    btn.type        = 'button';
    btn.textContent = '\u6362\u4e00\u9996';  // 换一首

    wrap.appendChild(deco);
    wrap.appendChild(lineEl);
    wrap.appendChild(srcEl);
    wrap.appendChild(btn);
    siteInfo.appendChild(wrap);
  }

  function applyPoem(item) {
    var wrap = document.querySelector('.jj-verse');
    if (!wrap) return;
    var lineEl = wrap.querySelector('.jj-verse-line');
    var srcEl  = wrap.querySelector('.jj-verse-src');
    // Fade out → swap text → fade in
    wrap.classList.add('jj-verse--out');
    setTimeout(function () {
      lineEl.textContent = item.line;
      srcEl.textContent  = item.source ? '\u2500\u2500\u2500 ' + item.source : '';
      wrap.classList.remove('jj-verse--out');
    }, 280);
  }

  function loadPoetry() {
    fetchPoetryFromAPI(function (item) {
      applyPoem(item || getFallbackPoem());
    });
  }

  // ── Bootstrap ─────────────────────────────────────────────────────────────
  function startDynamicHome() {
    if (!isHomePage()) return;

    buildHero();
    loadPoetry();

    var btn = document.querySelector('.jj-verse-btn');
    if (btn) {
      btn.addEventListener('click', function () {
        btn.disabled = true;
        loadPoetry();
        setTimeout(function () { btn.disabled = false; }, 700);
      });
    }

    // Auto-rotate every 60 s
    setInterval(loadPoetry, 60000);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', startDynamicHome);
  } else {
    startDynamicHome();
  }
})();
