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

  var seasonGroups = {
    spring: [
      { line: '等闲识得东风面，万紫千红总是春。', source: '朱熹《春日》' },
      { line: '春风又绿江南岸，明月何时照我还。', source: '王安石《泊船瓜洲》' },
      { line: '乱花渐欲迷人眼，浅草才能没马蹄。', source: '白居易《钱塘湖春行》' },
      { line: '竹外桃花三两枝，春江水暖鸭先知。', source: '苏轼《惠崇春江晚景》' }
    ],
    summer: [
      { line: '接天莲叶无穷碧，映日荷花别样红。', source: '杨万里《晓出净慈寺送林子方》' },
      { line: '小荷才露尖尖角，早有蜻蜓立上头。', source: '杨万里《小池》' },
      { line: '绿树阴浓夏日长，楼台倒影入池塘。', source: '高骈《山亭夏日》' },
      { line: '明月别枝惊鹊，清风半夜鸣蝉。', source: '辛弃疾《西江月》' }
    ],
    autumn: [
      { line: '晴空一鹤排云上，便引诗情到碧霄。', source: '刘禹锡《秋词》' },
      { line: '停车坐爱枫林晚，霜叶红于二月花。', source: '杜牧《山行》' },
      { line: '落霞与孤鹜齐飞，秋水共长天一色。', source: '王勃《滕王阁序》' },
      { line: '莫听穿林打叶声，何妨吟啸且徐行。', source: '苏轼《定风波》' }
    ],
    winter: [
      { line: '忽如一夜春风来，千树万树梨花开。', source: '岑参《白雪歌送武判官归京》' },
      { line: '千山鸟飞绝，万径人踪灭。', source: '柳宗元《江雪》' },
      { line: '墙角数枝梅，凌寒独自开。', source: '王安石《梅花》' },
      { line: '晚来天欲雪，能饮一杯无。', source: '白居易《问刘十九》' }
    ]
  };

  function getDailySeed() {
    var d = new Date();
    return d.getFullYear() * 1000 + (d.getMonth() + 1) * 50 + d.getDate();
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
    button.textContent = '今日诗词';

    wrap.appendChild(line);
    wrap.appendChild(source);
    wrap.appendChild(button);
    siteInfo.appendChild(wrap);

    return wrap;
  }

  function getSeasonByMonth(month) {
    if (month >= 2 && month <= 4) {
      return 'spring';
    }
    if (month >= 5 && month <= 7) {
      return 'summer';
    }
    if (month >= 8 && month <= 10) {
      return 'autumn';
    }
    return 'winter';
  }

  function preloadImage(url, onLoad) {
    var img = new Image();
    img.onload = function () {
      onLoad(url);
    };
    img.src = url;
  }

  function applyWallpaper(url) {
    var header = getHeaderElement();
    if (!header) {
      return;
    }

    header.style.backgroundImage = 'linear-gradient(120deg, rgba(3, 17, 36, 0.52), rgba(5, 24, 52, 0.36)), url(' + url + ')';
    header.style.backgroundSize = 'cover';
    header.style.backgroundPosition = 'center center';
    header.style.backgroundRepeat = 'no-repeat';
  }

  function applyClassic(item) {
    var wrap = createPoetryNode();
    if (!wrap) {
      return;
    }

    var lineNode = wrap.querySelector('.jj-hero-poetry-line');
    var sourceNode = wrap.querySelector('.jj-hero-poetry-source');
    lineNode.textContent = item.line;
    sourceNode.textContent = item.source;
  }

  function startDynamicHome() {
    if (!isHomePage()) {
      return;
    }

    var seed = getDailySeed();
    var wallpaperIndex = seed % wallpapers.length;
    var month = new Date().getMonth();
    var seasonKey = getSeasonByMonth(month);
    var seasonClassics = seasonGroups[seasonKey];
    var classicIndex = seed % seasonClassics.length;

    preloadImage(wallpapers[wallpaperIndex], applyWallpaper);
    applyClassic(seasonClassics[classicIndex]);

    var poetryWrap = createPoetryNode();
    if (poetryWrap) {
      var nextBtn = poetryWrap.querySelector('.jj-hero-poetry-btn');
      if (nextBtn && !nextBtn.dataset.bound) {
        nextBtn.dataset.bound = '1';
        nextBtn.addEventListener('click', function () {
          classicIndex = (classicIndex + 1) % seasonClassics.length;
          applyClassic(seasonClassics[classicIndex]);
        });
      }
    }

    window.setInterval(function () {
      wallpaperIndex = (wallpaperIndex + 1) % wallpapers.length;
      classicIndex = (classicIndex + 1) % seasonClassics.length;
      preloadImage(wallpapers[wallpaperIndex], applyWallpaper);
      applyClassic(seasonClassics[classicIndex]);
    }, 45000);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', startDynamicHome);
  } else {
    startDynamicHome();
  }
})();
