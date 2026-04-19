(function () {
  'use strict';

  var BLOCK_SELECTOR = '#article-container pre, #article-container figure.highlight';
  var BLOCK_CLASS = 'jj-codeblock';
  var DARK_CLASS = 'jj-code-dark';
  var BTN_CLASS = 'jj-code-theme-toggle';

  function hasCodeBlocks() {
    return Boolean(document.querySelector(BLOCK_SELECTOR));
  }

  function getButtonLabel(isDark) {
    return isDark ? 'Light' : 'Dark';
  }

  function ensureBlockClass(block) {
    if (!block.classList.contains(BLOCK_CLASS)) {
      block.classList.add(BLOCK_CLASS);
    }
  }

  function updateButton(button, block) {
    button.textContent = getButtonLabel(block.classList.contains(DARK_CLASS));
    button.setAttribute('aria-label', '切换当前代码块亮暗主题');
    button.setAttribute('title', '切换当前代码块亮暗主题');
  }

  function ensureButtonForBlock(block) {
    ensureBlockClass(block);

    var button = block.querySelector('.' + BTN_CLASS);
    if (!button) {
      button = document.createElement('button');
      button.className = BTN_CLASS;
      button.type = 'button';
      block.appendChild(button);
    }

    if (!button.dataset.bound) {
      button.dataset.bound = '1';
      button.addEventListener('click', function () {
        block.classList.toggle(DARK_CLASS);
        updateButton(button, block);
      });
    }

    updateButton(button, block);
  }

  function initCodeThemeButtons() {
    if (!hasCodeBlocks()) {
      return;
    }

    var blocks = document.querySelectorAll(BLOCK_SELECTOR);
    blocks.forEach(function (block) {
      ensureButtonForBlock(block);
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initCodeThemeButtons);
  } else {
    initCodeThemeButtons();
  }

  document.addEventListener('pjax:complete', initCodeThemeButtons);
})();