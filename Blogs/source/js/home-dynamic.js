(function () {
  'use strict';

  function isHomePage() {
    var path = window.location.pathname;
    return path === '/' || path === '/index.html';
  }

  function getHeaderElement() {
    return document.getElementById('page-header');
  }

  function getSiteInfoElement() {
    return document.getElementById('site-info');
  }

  var wallpapers = [
    'https://images.unsplash.com/photo-1506744038136-46273834b3fb?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1472214103451-9374bd1c798e?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1469474968028-56623f02e42e?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1441974231531-c6227db76b6e?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1493244040629-496f6d136cc3?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1500530855697-b586d89ba3ee?auto=format&fit=crop&w=1920&q=80'
  ];

  // 本地备用诗词池，API 失败时降级使用（覆盖多朝代经典）
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
    { line: '烽火连三月，家书抵万金。', source: '杜甫《春望》' }
  ];

  function getDailySeed() {
    var d = new Date();
    return d.getFullYear() * 1000 + (d.getMonth() + 1) * 50 + d.getDate();
  }

  // 调用今日诗词 API（接入 chinese-poetry 库，30 万+ 经典条目）
  function fetchPoetryFromAPI(callback) {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', 'https://v2.jinrishici.com/one.json', true);
    xhr.withCredentials = true;
    xhr.timeout = 5000;
    xhr.onreadystatechange = function () {
      if (xhr.readyState !== 4) return;
      if (xhr.status === 200) {
        try {
          var resp = JSON.parse(xhr.responseText);
          if (resp.status === 'success' && resp.data && resp.data.content) {
            var origin = resp.data.origin || {};
            var src = (origin.author ? origin.author : '') +
              (origin.title ? '《' + origin.title + '》' : '');
            callback({ line: resp.data.content, source: src });
            return;
          }
        } catch (e) {}
      }
      callback(null);
    };
    xhr.onerror = function () { callback(null); };
    xhr.ontimeout = function () { callback(null); };
    xhr.send();
  }

  function getFallbackPoem() {
    return fallbackPoems[getDailySeed() % fallbackPoems.length];
  }

  function createPoetryNode() {
    var existing = document.querySelector('.jj-hero-poetry');
    if (existing) {
      return existing;
    }

    var siteInfo = getSiteInfoElement();
    if (!siteInfo) {
      return null;
    }

    var wrap = document.createElement('div');
    wrap.className = 'jj-hero-poetry';

    var line = document.createElement('div');
    line.className = 'jj-hero-poetry-line';

    var source = document.createElement('div');
    source.className = 'jj-hero-poetry-source';

    var button = document.createElement('button');
    button.className = 'jj-hero-poetry-btn';
    button.type = 'button';
    button.textContent = '换一首';

    wrap.appendChild(line);
    wrap.appendChild(source);
    wrap.appendChild(button);
    siteInfo.appendChild(wrap);

    return wrap;
  }

  function applyWallpaper(url) {
    var header = getHeaderElement();
    if (!header) {
      return;
    }

    header.style.backgroundImage = 'linear-gradient(120deg, rgba(4, 13, 28, 0.72), rgba(10, 27, 54, 0.58)), url(' + url + ')';
    header.style.backgroundSize = 'cover';
    header.style.backgroundPosition = 'center center';
    header.style.backgroundRepeat = 'no-repeat';
  }

  function applyClassic(item) {
    var wrap = createPoetryNode();
    if (!wrap) return;
    wrap.querySelector('.jj-hero-poetry-line').textContent = item.line;
    wrap.querySelector('.jj-hero-poetry-source').textContent = item.source;
  }

  // 从今日诗词 API 获取，失败时降级本地备用
  function loadPoetry() {
    var wrap = createPoetryNode();
    if (wrap) {
      var lineNode = wrap.querySelector('.jj-hero-poetry-line');
      if (lineNode) lineNode.textContent = '正在加载诗词…';
    }
    fetchPoetryFromAPI(function (item) {
      applyClassic(item || getFallbackPoem());
    });
  }

  function startDynamicHome() {
    if (!isHomePage()) return;

    var seed = getDailySeed();
    var wallpaperIndex = seed % wallpapers.length;

    // 直接应用壁纸，不等预加载，消除 seele.jpg 闪烁
    applyWallpaper(wallpapers[wallpaperIndex]);

    // 从中华诗词 API 动态加载诗词
    loadPoetry();

    var poetryWrap = createPoetryNode();
    if (poetryWrap) {
      var nextBtn = poetryWrap.querySelector('.jj-hero-poetry-btn');
      if (nextBtn && !nextBtn.dataset.bound) {
        nextBtn.dataset.bound = '1';
        nextBtn.addEventListener('click', function () {
          loadPoetry();
        });
      }
    }

    window.setInterval(function () {
      wallpaperIndex = (wallpaperIndex + 1) % wallpapers.length;
      applyWallpaper(wallpapers[wallpaperIndex]);
      loadPoetry();
    }, 45000);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', startDynamicHome);
  } else {
    startDynamicHome();
  }
})();
